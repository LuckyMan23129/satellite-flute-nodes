/**
 * @file uart_sensor.c
 * @brief Ultrasonic Distance Sensor Driver (UART2)
 *
 * Reads water-level distance from an ultrasonic sensor connected to UART
 * at 9600 baud. Uses Zephyr async UART API with two dedicated semaphores:
 *
 *   uart_data_ready_sem  — given in UART_RX_DISABLED after uart_value[] has
 *                          been written in UART_RX_RDY. Used by
 *                          check_data_available() to signal that the RX cycle
 *                          has ended and sensor data is ready to read.
 *
 *   uart_rx_disabled_sem — given in UART_RX_DISABLED whenever the RX peripheral
 *                          stops (naturally or forced). Used by the retry path
 *                          and cleanup path to confirm uart_rx_disable() has
 *                          fully completed before re-enabling RX.
 *
 * Frame format: [0xFF, H, L, CS] (4 bytes)
 *   CS = (0xFF + H + L) & 0xFF
 *   Distance = (H << 8) | L  in mm
 *
 * @author DistriNet LAB, KU Leuven
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>
#include "uart_sensor.h"
#include "lowpower.h"
#include "config.h"

LOG_MODULE_REGISTER(uart_sensor, LOG_LEVEL_INF);

#define SLEEP_TIME_MS2      0
#define RECEIVE_BUFF_SIZE   8       /* 2× the 4-byte frame so we always capture at least one
                                     * complete frame even if RX starts mid-frame. The 0xFF
                                     * start byte is then found by scanning the buffer. */
#define RECEIVE_TIMEOUT     10000   /* µs — must be > 1 bit-period (104 µs @ 9600 baud)
                                     * and > full 4-byte frame time (~4.2 ms @ 9600 baud).
                                     * 100 µs was borderline: inter-byte idle ≈ 104 µs
                                     * caused per-byte RX_RDY events (len=1) that were
                                     * discarded by the len>1 guard. */
#define ENABLE_READ_TIMEOUT_MS  	50	// Timeout - To prevent infinite loop when reading UART data


static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
static uint8_t uart_value[4] = {0};
static uint16_t  old_distance = 0;

/* Accumulation buffer: collects bytes across fragmented RX_RDY events so a
 * complete 4-byte frame can be reconstructed even when the UART interrupt
 * fires with len < 4 (e.g. at startup when RX begins mid-frame).
 */
static uint8_t acc_buf[16];
static uint8_t acc_pos = 0;
static bool    frame_valid = false;  /* set when uart_value[] holds a received frame */

/* Signalled in UART_RX_DISABLED after uart_value[] has been written in UART_RX_RDY.
 * Giving the semaphore here (rather than in UART_RX_RDY) guarantees it fires
 * unconditionally at the end of every RX cycle so check_data_available() always
 * unblocks, even if the frame arrived in chunks (len < 4 per RDY event).
 */
K_SEM_DEFINE(uart_data_ready_sem, 0, 1);

/* Signalled in UART_RX_DISABLED whenever an RX cycle ends (naturally or forced).
 * Used exclusively by the retry path and cleanup path to confirm uart_rx_disable()
 * has fully completed before calling uart_rx_enable() again.
 */
K_SEM_DEFINE(uart_rx_disabled_sem, 0, 1);

/* Maximum number of UART reception retries before giving up.
 * Each retry takes ~80 ms
 */
#define UART_MAX_RETRIES  (10)

// Get the device pointer of the UART sensor hardware
const struct device *uart_dev= DEVICE_DT_GET(DT_NODELABEL(uart2));


/* uart_cb is only passed to uart_callback_set() — no external linkage needed */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	LOG_DBG("Callback function was just called");
	switch (evt->type) {
		case UART_RX_RDY: {
			uint8_t len = evt->data.rx.len;
			const uint8_t *buf = evt->data.rx.buf + evt->data.rx.offset;
			LOG_INF("RX_RDY len=%u buf[0]=0x%02X", len, len > 0 ? buf[0] : 0x00);

			/* Append incoming bytes to the accumulation buffer.
			 * This handles fragmented delivery (len < 4 per event) that
			 * occurs at startup when RX begins mid-frame.
			 */
			uint8_t copy_len = MIN(len, (uint8_t)(sizeof(acc_buf) - acc_pos));
			memcpy(&acc_buf[acc_pos], buf, copy_len);
			acc_pos += copy_len;

			/* Scan accumulated bytes for a complete frame: 0xFF + 3 bytes */
			for (uint8_t i = 0; i + 4 <= acc_pos; i++) {
				if (acc_buf[i] == 0xFF) {
					memcpy(uart_value, &acc_buf[i], 4);
					frame_valid = true;
					k_sem_give(&uart_data_ready_sem);
					acc_pos = 0;
					break;
				}
			}

			/* Buffer full with no valid frame — discard and resync */
			if (acc_pos >= sizeof(acc_buf)) {
				acc_pos = 0;
			}
		}
		break;

		case UART_RX_DISABLED: {
			/* Signal that the RX cycle has ended and uart_value[] is ready.
			 * Fired after UART_RX_RDY (data path) or after uart_rx_disable()
			 * (retry/cleanup path). uart_data_ready_sem is given here so
			 * check_data_available() unblocks regardless of how RX ended.
			 */
			k_sem_give(&uart_data_ready_sem);
			k_sem_give(&uart_rx_disabled_sem);
			LOG_DBG("UART_RX_DISABLED");
			}
		break;
		case UART_RX_STOPPED:
		    LOG_DBG("UART_RX_STOPPED");
		break;
		case UART_RX_BUF_RELEASED:
            LOG_DBG("UART_RX_BUF_RELEASED");
		break;
		case UART_RX_BUF_REQUEST:
            LOG_DBG("UART_RX_BUF_REQUEST");
		break;
		case UART_TX_DONE:
		    LOG_DBG("UART_TX_DONE");
		break;
		case UART_TX_ABORTED:
		    LOG_DBG("UART_TX_ABORTED");
		break;
		default:
			break;
	}
}


int8_t initialize_uart_sensor(void)
{
	int8_t ret;   /* local — no need to share across functions */

	// Verify that the UART device is ready
	if (!device_is_ready(uart_dev)){
		LOG_ERR("UART device not ready");
		return 1;
		}

	// Register the UART callback function
	ret = uart_callback_set(uart_dev, uart_cb, NULL);
		if (ret) {
			LOG_ERR("Fail to set callback function");
			return 1;
		}

	// Clear any stale signals and accumulation state from a previous reception cycle
	k_sem_reset(&uart_data_ready_sem);
	k_sem_reset(&uart_rx_disabled_sem);
	acc_pos = 0;
	frame_valid = false;

	// Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer
	ret = uart_rx_enable(uart_dev, rx_buf,sizeof rx_buf, RECEIVE_TIMEOUT);
	if (ret) {
		LOG_ERR("Fail to enable RX (err %d)", ret);
		return 1;
	}

	k_sleep(K_MSEC(60));

	return 0;
}


// Returns true when the RX cycle has ended and uart_value[] is ready to read
bool check_data_available(void)
{
	return (k_sem_take(&uart_data_ready_sem, K_MSEC(ENABLE_READ_TIMEOUT_MS)) == 0);
}


uint16_t read_distance(void)
{
	uint16_t distance_mm = 0;

	/* Sensor frame format: [0xFF, H, L, CS]  (4-byte, 0xFF start byte)
	 * uart_value[0] = 0xFF start byte
	 * uart_value[1] = distance high byte
	 * uart_value[2] = distance low byte
	 * uart_value[3] = checksum = (0xFF + H + L) & 0xFF
	 */
	if (uart_value[0] != 0xFF) {
		/* Only warn if a frame was actually received but is corrupt.
		 * Silence the log when uart_value[] is still zero-initialised
		 * (UART_RX_DISABLED fired before any valid frame arrived).
		 */
		if (frame_valid) {
			LOG_WRN("Bad start byte 0x%02X — discarding frame", uart_value[0]);
		}
		return 0;
	}

	if(((0xFF + uart_value[1] + uart_value[2]) & 0xFF) == uart_value[3])
	{
		distance_mm = ((uint16_t)uart_value[1] << 8) | uart_value[2];
		if(distance_mm > 280)
    	{
			return (distance_mm);
    	}
		else{
       		LOG_WRN("Below the lowest limit of sensor (%u mm)", distance_mm);
			return (0);
  		}
	}
	LOG_WRN("Checksum mismatch: [0x%02X 0x%02X 0x%02X 0x%02X] — discarding frame",
	       uart_value[0], uart_value[1], uart_value[2], uart_value[3]);
	return (0);
}


uint16_t distance_cm(void)
{
	// 1. Turn on UART for reading sensor data
	lowpower_setup_uart2_ENA();
	// lowpower_setup_uart0_ENA();
	k_sleep(K_MSEC(20));

    // 2. Initialize ASYN UART to read data
    initialize_uart_sensor();

    // Wait for an incoming UART message, retrying up to UART_MAX_RETRIES times.
    // Without this limit, a broken/disconnected sensor would cause an infinite
    // loop, preventing the MCU from ever returning to sleep — fatal for a
    // battery-free device.
    uint32_t retries = 0;
    uint16_t read_distance_cm = 0;

    while (retries < UART_MAX_RETRIES)
    {
      bool data_ready = check_data_available();
      read_distance_cm = read_distance() / 10;

      if (data_ready && read_distance_cm > 0) {
        break;  /* valid frame received */
      }

      retries++;
      if (retries < UART_MAX_RETRIES) {
        /* Stop RX cleanly before re-initializing */
        uart_rx_disable(uart_dev);
        k_sem_take(&uart_rx_disabled_sem, K_MSEC(50));
        k_sem_reset(&uart_rx_disabled_sem);
        initialize_uart_sensor();
      }
    }

    if (read_distance_cm == 0) {
        LOG_WRN("No valid frame after %u retries, returning 0", retries);
    }
	/* Disable async RX cleanly before suspending the device.
	 * This is necessary in the retry-exhausted path where RX may still be
	 * active; calling PM_DEVICE_ACTION_SUSPEND while RX is running returns
	 * -EBUSY and the suspend silently fails.
	 */
	uart_rx_disable(uart_dev);
	k_sem_take(&uart_rx_disabled_sem, K_MSEC(50));   /* wait for UART_RX_DISABLED */
	k_sem_reset(&uart_rx_disabled_sem);

	// Disable UART to save the energy
	lowpower_setup_uart2_DIS();

    // Read sensor data
    return (read_distance_cm);     // Distance is measured in Cm

}

uint16_t get_safe_distance_cm(void)
{
	uint16_t  distance = 0;
	distance = distance_cm();
	// Prevent wrong readings
    if (abs((int16_t)old_distance - (int16_t)distance) >= 15)
    {
    	distance = distance_cm();
    }

    old_distance = distance;
    if (ENABLE_PRINT){
        LOG_INF("Water Level: %d cm", distance);
	}

	return (distance);
}