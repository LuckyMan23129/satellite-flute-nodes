/*
 * main.c — Iridium satellite modem application entry point.
 *
 * Frame format (10 bytes):
 *   [0-1]:  Sleep time (16-bit big-endian)
 *   [2-3]:  Vcap (16-bit big-endian)
 *   [4-5]:  Vpv (16-bit big-endian)
 *   [6-7]:  Sensor reading (16-bit big-endian)
 *   [8-9]:  Signal quality from modem (16-bit big-endian)
 *
 * Total: 10 bytes of data (no checksum - modem adds its own)
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/byteorder.h>

// Modem libraries for Low power
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

#include "config.h"
#include "astar_diurnal.h"
#include "read_Vcap.h"
#include "read_Vsupply.h"
#include "switch_Vpv_divider.h"
#include "switch_opencircuit.h"
#include "switch_peripheral_power.h"
#include "lowpower.h"
#include "i2c_sensor4.h"
#include "iridium_uart.h"

LOG_MODULE_REGISTER(iridium_main, LOG_LEVEL_INF);

const bool ENABLE_PRINT = true;

static uint16_t distance;
static uint16_t newV;
static uint32_t sleep_duration;
static uint16_t Vpv;

uint8_t frame[FRAME_SIZE];
static const iridium_t *satellite;
static void build_frame(uint8_t *frame);

// ---------------------------------------------------------------------------
// AsTAR++ Configuration
// ---------------------------------------------------------------------------
/**
 * @brief AsTAR++ algorithm configuration parameters.
 *
 * Modify these values to tune the algorithm for your specific hardware and
 * application requirements.
 */
static const astar_config_t astar_params = {
    .USE_BOOST = false,

    .maxVoltage               = 5100,
    .shutOffVoltage           = 3300,
    .daytimeOptimumV          = 4900,
    .OpenCircuitVoltage       = 5400,
    .wakeup_Vpv_Threshold     = 4000,
    .sleep_Vpv_Threshold      = 3800,
    .check_overVcap_threshold = 3900,

    .maxRate                        = 120,
    .minRate                        = 7200,
    .nighttimeMaxRate               = 120,
    .LowVolt_SleepTime              = 7200,
    .daily_initial_wakeup_SleepTime = 600,
    .daily_initial_sleep_SleepTime  = 300,

    .nightDurationRollingEstimate = 48000,
};


// ---------------------------------------------------------------------------
// Overvoltage Protection Thread
// ---------------------------------------------------------------------------
#define OVER_V_STACK_SIZE 2048
#define OVER_V_PRIORITY   4
K_THREAD_DEFINE(over_v_id, OVER_V_STACK_SIZE, overV_protection_thread,
                NULL, NULL, NULL, OVER_V_PRIORITY, 0, 0);

// ---------------------------------------------------------------------------#
// Power-Rail Protection                                                      #
// ---------------------------------------------------------------------------#
/**
 * @brief Mutex that serialises modem transmission against solar-panel
 *        reconnection.  Hold it while calling modem_transmitData(); the
 *        overV protection thread acquires it before enable_charging() and
 *        releases it only after the rail has stabilised.
 */
K_MUTEX_DEFINE(modem_rail_mutex_0);


/* -------------------------------------------------------------------------
 * Main entry point
 * ---------------------------------------------------------------------- */
int main(void)
{   
    if (ENABLE_PRINT)
        LOG_INF("Entering main function .....");

    // ── Low-power peripheral setup ────────────────────────────────────────
    lowpower_setup_gpio();
    lowpower_setup_accel();
    // lowpower_setup_uart0_DIS();

    // ── AsTAR++ initialisation ────────────────────────────────────────────
    astar_init(&astar_params);

    // ── GPIO checks ───────────────────────────────────────────────────────
    if (check_gpio_sw_opencircuit()) {
        LOG_INF("Open-circuit switch GPIO not found in Devicetree");
        k_sleep(K_FOREVER);
    }

    if (check_gpio_Vpv_divider()) {
        LOG_INF("Vpv divider switch GPIO not found in Devicetree");
        k_sleep(K_FOREVER);
    }

    if (check_gpio_periph_sw()) {
        LOG_INF("Peripheral power switch GPIO not found in Devicetree");
        k_sleep(K_FOREVER);
    }

    // Disable Vpv divider to save energy during modem connection
    disable_Vpv_divider();
    
    // Init cellular modem - low-power sleep
	nrf_modem_lib_init();
	lte_lc_init();

    // Iridium UART setup
    satellite = iridium_get();

    while (1) {
        //====================================================================
        // 1. Voltage check — deep sleep immediately if Vcap is too low
        //====================================================================
    rerun_astar_after_suspension:
        // Read current Vcap
        newV = astar_safe_read_vcap();
        // Update internal state
        astar_safe_update_vcap(newV);

        if (astar_should_suspend()) {
            setSuspensionHandler();
            goto rerun_astar_after_suspension;
        }

        //========================================================================
        // Iridium UART setup
        //========================================================================
        periph_sw_turn_on();
        k_sleep(K_SECONDS(20));
        lowpower_setup_uart2_ENA();

        LOG_INF("Initializing Iridium UART driver...");
        int ret = satellite->uart_init();

        if (ret != 0) {
            LOG_ERR("satellite->uart_init() failed: %d — powering down and retrying next cycle", ret);
            lowpower_setup_uart2_DIS();
            periph_sw_turn_off();
            k_sleep(K_SECONDS(sleep_duration));
            continue;
        }

        LOG_INF("Iridium UART driver initialized successfully");

        //====================================================================
        // 2. LiDAR sensor reading
        //====================================================================
        k_sleep(K_SECONDS(20));
        distance = lidar_read_water_level();
        // periph_sw_turn_off();

        //====================================================================
        // 3. AsTAR++ scheduler
        //====================================================================
        if (ENABLE_PRINT)
            LOG_INF("Running AsTAR++ scheduler...");
        sleep_duration = schedule();

        //====================================================================
        // 4. Prepare data frame
        //====================================================================
        Vpv = astar_get_vpv_mv();
        build_frame(frame);
        
        //====================================================================
        // 5. Prepare data frame and transmit it to server
        //====================================================================
        // k_mutex_lock(&modem_rail_mutex_0, K_FOREVER); 
        satellite->send_binary(frame, FRAME_SIZE);
        // k_mutex_unlock(&modem_rail_mutex_0);
        
        //====================================================================
        // 6. Power down peripherals and suspend UART before sleep
        //====================================================================
        lowpower_setup_uart2_DIS();
        periph_sw_turn_off();
        
        //====================================================================
        // 7. Deep sleep
        //====================================================================
        if (ENABLE_PRINT)
            LOG_INF("Sleeping for %d s", sleep_duration);
        k_sleep(K_SECONDS(sleep_duration));    
        // k_sleep(K_FOREVER);  // Sleep indefinitely after one transmission for testing
    }

    return 0;
}



/*==========================================================================================================*/
/* -------------------------------------------------------------------------
 * Frame helpers
 * ---------------------------------------------------------------------- */

/* Reads sensor and signal quality, packs all values into the 10-byte
 * big-endian frame that will be transmitted to the satellite. */
static void build_frame(uint8_t *frame)
{
    int signal_quality = satellite->get_rssi();
    if (signal_quality < 0 || signal_quality > 99) {
        LOG_WRN("Invalid signal quality %d, using 99", signal_quality);
        signal_quality = 99;
    }

    uint16_t sleep_time_be = sys_cpu_to_be16((uint16_t)sleep_duration);
    uint16_t vcap_be       = sys_cpu_to_be16((uint16_t)newV);
    uint16_t vpv_be        = sys_cpu_to_be16((uint16_t)Vpv);
    uint16_t sensor_be     = sys_cpu_to_be16((uint16_t)distance);
    uint16_t signal_be     = sys_cpu_to_be16((uint16_t)signal_quality);

    frame[0] = (uint8_t)(sleep_time_be >> 8);
    frame[1] = (uint8_t)(sleep_time_be & 0xFF);
    frame[2] = (uint8_t)(vcap_be >> 8);
    frame[3] = (uint8_t)(vcap_be & 0xFF);
    frame[4] = (uint8_t)(vpv_be >> 8);
    frame[5] = (uint8_t)(vpv_be & 0xFF);
    frame[6] = (uint8_t)(sensor_be >> 8);
    frame[7] = (uint8_t)(sensor_be & 0xFF);
    frame[8] = (uint8_t)(signal_be >> 8);
    frame[9] = (uint8_t)(signal_be & 0xFF);

    LOG_INF("Frame built (%u bytes):", FRAME_SIZE);
    LOG_INF("  Sleep_time: %u sec", sleep_duration);
    LOG_INF("  Vcap:       %u mV", newV);
    LOG_INF("  Vpv:        %u mV", Vpv);
    LOG_INF("  Sensor:     %u", distance);
    LOG_INF("  Signal:     %d", signal_quality);
    LOG_DBG("Frame bytes:");
    LOG_DBG("  [0-4]: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            frame[0], frame[1], frame[2], frame[3], frame[4]);
    LOG_DBG("  [5-9]: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            frame[5], frame[6], frame[7], frame[8], frame[9]);
}