/*
 * iridium_uart.c — UART transport implementation for Iridium 9602/9603 modem
 *
 * SEMAPHORE VERSION — Event-driven with zero polling overhead.
 *
 * See iridium_uart.h for the public API.
 */

#include "iridium_uart.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>   /* sys_cpu_to_be16() */
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "config.h"

LOG_MODULE_REGISTER(iridium_uart, LOG_LEVEL_DBG);

/* Forward declaration — defined later in this file. */
static const char *iridium_result_str(iridium_result_t result);

/* -------------------------------------------------------------------------
 * Hardware binding
 * ---------------------------------------------------------------------- */

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart2));

/* -------------------------------------------------------------------------
 * Buffer definitions
 * ---------------------------------------------------------------------- */

#define RX_BUF_SIZE   256U
/*
 * Two RX buffers for Zephyr async double-buffering: while the driver fills
 * one, the callback immediately hands the other back via UART_RX_BUF_REQUEST
 * so reception is never interrupted.
 */
static uint8_t rx_buf[2][RX_BUF_SIZE];
static uint8_t active_rx_buf;

/*
 * Binary TX buffer: large enough for the maximum SBD payload plus the
 * 2-byte checksum appended by iridium_sbd_write_binary().
 */
static uint8_t bin_tx_buf[IRIDIUM_SBD_MAX_BINARY_LEN + 2U];

/* -------------------------------------------------------------------------
 * Response accumulator
 *
 * Written from uart_cb() (callback context).
 * Read from thread context in command functions.
 * Cleared atomically via irq_lock() before each new command.
 * ---------------------------------------------------------------------- */

static char   resp_buf[IRIDIUM_RESP_MAX_LEN];
static size_t resp_len;

/* -------------------------------------------------------------------------
 * Synchronization primitives
 * ---------------------------------------------------------------------- */

/*
 * Semaphore signaled by UART callback when a complete response is detected.
 * Command functions block on this semaphore instead of polling.
 */
static struct k_sem response_sem;

/*
 * Result of the last response detection (set by UART callback).
 * Protected by the semaphore — only read after sem is signaled.
 */
static volatile iridium_result_t response_result;

/*
 * What substring the UART callback should look for (set by command functions).
 * When this string is found in resp_buf, callback signals the semaphore.
 */
static const char *volatile expected_response;

/*
 * TX completion semaphore — signaled by UART_TX_DONE/UART_TX_ABORTED callback,
 * taken by uart_tx_and_wait_txdone() in thread context.
 */
static struct k_sem tx_done_sem;

/* -------------------------------------------------------------------------
 * UART async callback
 * ---------------------------------------------------------------------- */

static void uart_cb(const struct device *dev,
                    struct uart_event   *evt,
                    void                *user_data)
{
    ARG_UNUSED(user_data);

    switch (evt->type) {

    case UART_TX_DONE:
        k_sem_give(&tx_done_sem);
        break;

    case UART_RX_RDY: {
        const uint8_t *src = &evt->data.rx.buf[evt->data.rx.offset];
        size_t         len = evt->data.rx.len;

        /* Append to accumulator, guarding against overflow. */
        if (len > 0U && (resp_len + len) < sizeof(resp_buf) - 1U) {
            memcpy(&resp_buf[resp_len], src, len);
            resp_len += len;
            resp_buf[resp_len] = '\0';
        } else if (len > 0U) {
            LOG_WRN("Response buffer full — %zu bytes dropped", len);
        }

        LOG_DBG("RX [%zu bytes]: %.*s", len, (int)len, (const char *)src);

        /* ---------------------------------------------------------------
         * EVENT-DRIVEN RESPONSE DETECTION
         * 
         * Check if we now have a complete response. If so, signal the
         * semaphore to wake the waiting thread immediately.
         * --------------------------------------------------------------- */

        if (expected_response != NULL) {
            /* Check for expected success response */
            if (strstr(resp_buf, expected_response) != NULL) {
                LOG_DBG("Response matched: '%s'", expected_response);
                response_result = IRIDIUM_OK;
                k_sem_give(&response_sem);   /* Wake waiting thread! */
                expected_response = NULL;     /* One-shot detection */
            }
            /* Check for error response */
            else if (strstr(resp_buf, "ERROR") != NULL) {
                LOG_ERR("Modem returned ERROR");
                response_result = IRIDIUM_ERROR;
                k_sem_give(&response_sem);   /* Wake waiting thread! */
                expected_response = NULL;     /* One-shot detection */
            }
        }

        break;
    }

    case UART_RX_BUF_REQUEST:
        /*
         * Hand the driver the OTHER buffer. This is the key double-buffer
         * handshake: the driver calls this event before the current buffer
         * is full so that reception is never interrupted.
         */
        active_rx_buf ^= 1U;
        (void)uart_rx_buf_rsp(dev, rx_buf[active_rx_buf], RX_BUF_SIZE);
        break;

    case UART_RX_DISABLED:
        /*
         * The driver ran out of buffer space (or was explicitly stopped).
         * Re-enable on the inactive buffer; do NOT touch the active one.
         */
        {
            uint8_t next = active_rx_buf ^ 1U;
            memset(rx_buf[next], 0, RX_BUF_SIZE);
            active_rx_buf = next;
            (void)uart_rx_enable(dev, rx_buf[active_rx_buf],
                                  RX_BUF_SIZE, 1000);
        }
        break;

    case UART_TX_ABORTED:
        LOG_ERR("UART TX aborted unexpectedly");
        k_sem_give(&tx_done_sem);   /* unblock the waiting thread to avoid a deadlock */
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */


/* -------------------------------------------------------------------------
 * Public API implementation
 * ---------------------------------------------------------------------- */

/* Registers the async UART callback, enables double-buffered RX, and waits
 * 10 s for the Iridium module to finish booting. */
int iridium_uart_init(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART2 device not ready (check DTS / overlay)");
        return -ENODEV;
    }

    LOG_DBG("UART2 device: %p", uart_dev);

    /* Initialize semaphores for event-driven TX and RX detection */
    k_sem_init(&response_sem, 0, 1);
    k_sem_init(&tx_done_sem, 0, 1);

    int ret = uart_callback_set(uart_dev, uart_cb, NULL);
    if (ret != 0) {
        LOG_ERR("uart_callback_set() failed: %d", ret);
        return ret;
    }

    active_rx_buf = 0U;
    ret = uart_rx_enable(uart_dev, rx_buf[active_rx_buf],
                         RX_BUF_SIZE, 1000);
    if (ret != 0) {
        LOG_ERR("uart_rx_enable() failed: %d", ret);
        return ret;
    }

    LOG_INF("Iridium UART initialised (semaphore mode) — waiting 10 s for module boot");
    k_sleep(K_SECONDS(10));

    return 0;
}


/* Clears the response accumulator under IRQ lock so stale data from a
 * previous command cannot trigger the next response check. */
static void resp_buf_reset(void)
{
    unsigned int key = irq_lock();
    memset(resp_buf, 0, sizeof(resp_buf));
    resp_len = 0U;
    irq_unlock(key);
}


/* Computes the Iridium SBD 16-bit checksum (simple sum of all data bytes). */
static uint16_t sbd_checksum(const uint8_t *data, size_t len)
{
    uint16_t sum = 0U;

    for (size_t i = 0U; i < len; i++) {
        sum += data[i];
    }

    return sum;
}


/* Sends raw bytes over UART and blocks on a semaphore until UART_TX_DONE (2 s timeout). */
static int uart_tx_and_wait_txdone(const uint8_t *buf, size_t len)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    k_sem_reset(&tx_done_sem);

    int ret = uart_tx(uart_dev, buf, len, SYS_FOREVER_US);
    if (ret != 0) {
        LOG_ERR("uart_tx() failed: %d", ret);
        return ret;
    }

    /* Block until UART_TX_DONE or UART_TX_ABORTED — 2 s timeout. */
    ret = k_sem_take(&tx_done_sem, K_SECONDS(2));
    if (ret != 0) {
        LOG_ERR("TX timeout — UART_TX_DONE not received");
        return -ETIMEDOUT;
    }

    return 0;
}




/* Sends an AT command and blocks on a semaphore until the expected response
 * substring, "ERROR", or the timeout is received. */
iridium_result_t iridium_send_cmd_verify_response(const char *cmd,
                                   const char *expected_resp,
                                   int         timeout_s)
{
    if (cmd == NULL || expected_resp == NULL || timeout_s <= 0) {
        LOG_ERR("iridium_send_cmd_verify_response(): invalid argument");
        return IRIDIUM_BAD_ARG;
    }

    size_t cmd_len = strlen(cmd);

    /* Reset response buffer and semaphore */
    resp_buf_reset();
    k_sem_reset(&response_sem);
    
    /* Tell UART callback what to look for */
    expected_response = expected_resp;
    response_result = IRIDIUM_TIMEOUT;  /* default if timeout occurs */

    /* Brief settling delay to let any in-flight bytes drain */
    k_sleep(K_MSEC(100));

    LOG_INF("CMD: %.*s", (int)(cmd_len > 0U ? cmd_len - 1U : 0U), cmd);

    /* Transmit the command */
    LOG_DBG("TX cmd [%zu bytes]: %.*s", cmd_len, (int)(cmd_len - 1U), cmd);
    int ret = uart_tx_and_wait_txdone((const uint8_t *)cmd, cmd_len);
    if (ret != 0) {
        LOG_ERR("Transmit failed: %d", ret);
        expected_response = NULL;
        return IRIDIUM_TX_FAIL;
    }

    /* -----------------------------------------------------------------------
     * EVENT-DRIVEN WAIT
     * 
     * Block on semaphore until UART callback detects the expected response
     * or timeout expires. Thread sleeps here with ZERO polling overhead.
     * ----------------------------------------------------------------------- */

    LOG_DBG("Waiting for response (blocking on semaphore, timeout %d s)...", timeout_s);
    
    ret = k_sem_take(&response_sem, K_SECONDS(timeout_s));

    expected_response = NULL;  /* Clear regardless of outcome */

    if (ret == 0) {
        /* Semaphore was signaled — callback detected response */
        LOG_INF("Response received: %s", iridium_result_str(response_result));
        return response_result;
    } else {
        /* Timeout expired */
        LOG_WRN("Timeout after %d s", timeout_s);
        return IRIDIUM_TIMEOUT;
    }
}


/* Loads a binary payload into the modem SBD TX buffer via AT+SBDWB,
 * appending the required 2-byte big-endian checksum automatically. */
iridium_result_t iridium_sbd_write_binary(const uint8_t *data,
                                           size_t         len,
                                           int            timeout_s)
{
    /* --- Argument validation --- */
    if (data == NULL || len == 0U || len > IRIDIUM_SBD_MAX_BINARY_LEN) {
        LOG_ERR("iridium_sbd_write_binary(): invalid argument "
                "(data=%p, len=%zu, max=%u)",
                data, len, IRIDIUM_SBD_MAX_BINARY_LEN);
        return IRIDIUM_BAD_ARG;
    }

    if (timeout_s <= 0) {
        LOG_ERR("iridium_sbd_write_binary(): timeout must be > 0");
        return IRIDIUM_BAD_ARG;
    }

    /* ------------------------------------------------------------------ */
    /* Phase 1 — Send "AT+SBDWB=<len>\r" and wait for "READY"             */
    /* ------------------------------------------------------------------ */

    char length_cmd[32];
    int  written = snprintf(length_cmd, sizeof(length_cmd),
                            "AT+SBDWB=%zu\r", len);

    if (written < 0 || (size_t)written >= sizeof(length_cmd)) {
        LOG_ERR("Failed to format AT+SBDWB command");
        return IRIDIUM_TX_FAIL;
    }

    LOG_INF("CMD: AT+SBDWB=%zu (%zu data bytes + 2-byte checksum)", len, len);

    iridium_result_t phase1 = iridium_send_cmd_verify_response(length_cmd, "READY", timeout_s);
    if (phase1 != IRIDIUM_OK) {
        LOG_ERR("AT+SBDWB command failed (%s)", iridium_result_str(phase1));
        return phase1;
    }

    /* ------------------------------------------------------------------ */
    /* Phase 2 — Send binary data + 2-byte big-endian checksum            */
    /* ------------------------------------------------------------------ */

    uint16_t cs    = sbd_checksum(data, len);
    uint16_t cs_be = sys_cpu_to_be16(cs);

    memcpy(bin_tx_buf, data, len);
    memcpy(&bin_tx_buf[len], &cs_be, sizeof(cs_be));

    size_t total_len = len + sizeof(cs_be);

    LOG_DBG("Sending %zu bytes of data + 2-byte checksum (0x%04X)", len, cs);

    resp_buf_reset();
    k_sem_reset(&response_sem);
    expected_response = "OK";
    response_result = IRIDIUM_TIMEOUT;

    int ret = uart_tx_and_wait_txdone(bin_tx_buf, total_len);
    if (ret != 0) {
        LOG_ERR("Failed to send binary payload: %d", ret);
        expected_response = NULL;
        return IRIDIUM_TX_FAIL;
    }

    /* ------------------------------------------------------------------ */
    /* Phase 3 — Wait for result code and interpret it                     */
    /* ------------------------------------------------------------------ */

    LOG_DBG("Waiting for binary write result (semaphore)...");
    ret = k_sem_take(&response_sem, K_SECONDS(timeout_s));
    expected_response = NULL;

    if (ret != 0) {
        LOG_ERR("AT+SBDWB result timeout");
        return IRIDIUM_TIMEOUT;
    }

    if (response_result == IRIDIUM_ERROR) {
        LOG_ERR("AT+SBDWB returned ERROR");
        return IRIDIUM_ERROR;
    }

    if (response_result != IRIDIUM_OK) {
        return response_result;
    }

    /* Parse the digit before "OK" */
    if (strstr(resp_buf, "0") != NULL) {
        LOG_INF("Binary payload accepted by modem (%zu bytes)", len);
        return IRIDIUM_OK;
    }

    if (strstr(resp_buf, "1") != NULL) {
        LOG_ERR("Modem rejected binary payload — checksum mismatch");
        return IRIDIUM_CHKSUM;
    }

    if (strstr(resp_buf, "2") != NULL) {
        LOG_ERR("Modem rejected binary payload — length mismatch");
        return IRIDIUM_ERROR;
    }

    LOG_ERR("Unexpected response after AT+SBDWB: %s", resp_buf);
    return IRIDIUM_ERROR;
}


/* Triggers a satellite SBD exchange (AT+SBDIX) to transmit the content
 * currently loaded in the modem TX buffer. Snapshots resp_buf under irq_lock
 * before parsing the MO status field to avoid a race with uart_cb(). */
iridium_result_t iridium_sbd_initiate_session(int timeout_s)
{
    static const char cmd[] = "AT+SBDIX\r";

    LOG_INF("CMD: AT+SBDIX (timeout %d s)", timeout_s);
    LOG_WRN("Satellite acquisition may take up to %d s", timeout_s);

    iridium_result_t result = iridium_send_cmd_verify_response(cmd, "OK", timeout_s);

    if (result != IRIDIUM_OK) {
        LOG_INF("AT+SBDIX completed: %s", iridium_result_str(result));
        return result;
    }

    /* Parse the MO status from "+SBDIX: <mo_status>, ..." */
    char local_buf[IRIDIUM_RESP_MAX_LEN];
    unsigned int key = irq_lock();
    memcpy(local_buf, resp_buf, sizeof(local_buf));
    irq_unlock(key);

    char *sbdix_line = strstr(local_buf, "+SBDIX:");
    if (sbdix_line == NULL) {
        LOG_WRN("No +SBDIX: line in response — cannot verify delivery");
        LOG_INF("AT+SBDIX completed: %s", iridium_result_str(result));
        return result;
    }

    int mo_status = -1;
    if (sscanf(sbdix_line, "+SBDIX: %d,", &mo_status) != 1) {
        LOG_WRN("Failed to parse +SBDIX MO status field");
        LOG_INF("AT+SBDIX completed: %s", iridium_result_str(result));
        return result;
    }

    if (mo_status == 0) {
        LOG_INF("+SBDIX MO status 0: message transferred successfully");
        LOG_INF("AT+SBDIX completed: OK");
        return IRIDIUM_OK;
    }

    /* Non-zero MO status means the message was NOT delivered */
    const char *mo_str =
        mo_status == 1  ? "MO transfer error" :
        mo_status == 2  ? "No response from gateway" :
        mo_status == 3  ? "Session did not complete" :
        mo_status == 4  ? "Invalid sequence number" :
        mo_status == 10 ? "GSS call did not complete" :
        mo_status == 12 ? "Message has too many segments" :
        mo_status == 13 ? "Session already in progress" :
        mo_status == 15 ? "Access denied" :
        mo_status == 16 ? "SBD blocked" :
        mo_status == 32 ? "No network service" :
        mo_status == 35 ? "Antenna fault" :
        mo_status == 36 ? "Radio disabled" :
        mo_status == 37 ? "ISU busy" :
        mo_status == 38 ? "Try again later (wait 3 min)" :
        mo_status == 40 ? "GSS SBD system problem" :
        "Unknown error";

    LOG_ERR("+SBDIX MO status %d: %s — message NOT delivered", mo_status, mo_str);

    result = (mo_status == 32) ? IRIDIUM_NO_NETWORK : IRIDIUM_ERROR;
    LOG_INF("AT+SBDIX completed: %s", iridium_result_str(result));
    return result;
}


/* -------------------------------------------------------------------------
 * Transmission cycle
 * ---------------------------------------------------------------------- */

/* Writes the pre-built frame to the modem SBD TX buffer and initiates
 * a satellite exchange; logs the outcome. */
static iridium_result_t transmit_data_frame(const uint8_t *frame,
                                             size_t         len)
{
    LOG_INF("=== Starting test transmission ===");
    LOG_INF("Writing %zu bytes to SBD TX buffer (AT+SBDWB)...", len);

    iridium_result_t result = iridium_sbd_write_binary(frame, len,
                                                        TIMEOUT_SBDWB_S);
    if (result != IRIDIUM_OK) {
        LOG_ERR("AT+SBDWB failed (%s)", iridium_result_str(result));
        return result;
    }

    LOG_INF("Data successfully written to modem");
    LOG_INF("Initiating satellite exchange (AT+SBDIX)...");
    LOG_WRN("Satellite acquisition may take up to %d s", TIMEOUT_SBDIX_S);

    result = iridium_sbd_initiate_session(TIMEOUT_SBDIX_S);

    if (result == IRIDIUM_OK) {
        LOG_INF("Satellite transmission successful!");
    } else {
        LOG_ERR("Satellite transmission failed (%s)",
                iridium_result_str(result));
        LOG_INF("Hint: a clear sky view is required for satellite contact");
    }

    return result;
}

/* Sends AT ping and disables flow control; shared by all transmission functions. */
static iridium_result_t modem_prepare(void)
{
    LOG_INF("Testing basic modem communication (AT)...");

    iridium_result_t result = iridium_send_cmd_verify_response("AT\r", "OK", TIMEOUT_AT_S);
    if (result != IRIDIUM_OK) {
        LOG_ERR("Modem not responding (%s)", iridium_result_str(result));
        LOG_ERR("Check modem power, UART connections, and configuration");
        return result;
    }

    LOG_INF("Disabling flow control (AT&K0)...");
    result = iridium_send_cmd_verify_response("AT&K0\r", "OK", TIMEOUT_AT_S);
    if (result != IRIDIUM_OK) {
        LOG_ERR("Failed to disable flow control (%s)", iridium_result_str(result));
        return result;
    }

    LOG_INF("Modem is responding correctly");
    return IRIDIUM_OK;
}

/* Verifies modem communication with AT, then transmits the supplied frame
 * to the satellite via transmit_data_frame(). */
void iridium_send_binary_frame(const uint8_t *frame, size_t len)
{
    LOG_INF("========================================");
    LOG_INF("  Iridium Data Transmission Cycle");
    LOG_INF("========================================");

    iridium_result_t result = modem_prepare();
    if (result != IRIDIUM_OK) {
        return;
    }

    result = transmit_data_frame(frame, len);

    if (result == IRIDIUM_OK) {
        LOG_INF("Test cycle completed successfully");
    } else {
        LOG_ERR("Test cycle failed with result: %s",
                iridium_result_str(result));
    }
}



/* Returns a human-readable string for a given iridium_result_t code. */
static const char *iridium_result_str(iridium_result_t result)
{
    switch (result) {
    case IRIDIUM_OK:       return "OK";
    case IRIDIUM_ERROR:    return "ERROR (modem)";
    case IRIDIUM_TIMEOUT:  return "TIMEOUT";
    case IRIDIUM_TX_FAIL:  return "TX_FAIL";
    case IRIDIUM_INIT_ERR: return "INIT_ERR";
    case IRIDIUM_BAD_ARG:  return "BAD_ARG";
    case IRIDIUM_CHKSUM:      return "CHECKSUM_MISMATCH";
    case IRIDIUM_NO_NETWORK:  return "NO_NETWORK";
    default:                  return "UNKNOWN";
    }
}


/* Queries signal quality via AT+CSQ and returns the RSSI value (0–31),
 * or 99 if the signal is undetectable or the command fails. Snapshots
 * resp_buf under irq_lock before parsing to avoid a race with uart_cb(). */
int iridium_get_signal_quality(void)
{
    static const char cmd[] = "AT+CSQ\r";

    LOG_INF("Reading signal quality (AT+CSQ)...");

    iridium_result_t result = iridium_send_cmd_verify_response(cmd, "OK", 10);
    if (result != IRIDIUM_OK) {
        LOG_WRN("AT+CSQ failed (%s), returning 99 (unknown)",
                iridium_result_str(result));
        return 99;
    }

    /* Parse response: +CSQ:<rssi>,<ber> */
    char local_buf[IRIDIUM_RESP_MAX_LEN];
    unsigned int key = irq_lock();
    memcpy(local_buf, resp_buf, sizeof(local_buf));
    irq_unlock(key);

    char *csq_line = strstr(local_buf, "+CSQ:");
    if (csq_line == NULL) {
        LOG_WRN("No +CSQ: response found, returning 99 (unknown)");
        return 99;
    }

    /* Extract RSSI value */
    int rssi = 99;
    int ber = 99;

    if (sscanf(csq_line, "+CSQ:%d,%d", &rssi, &ber) >= 1) {
        LOG_INF("Signal quality: RSSI=%d, BER=%d", rssi, ber);

        /* Validate and log signal strength */
        if (rssi >= 0 && rssi <= 31) {
            LOG_INF("  Signal strength: %s",
                    rssi == 0 ? "Very weak (-113 dBm or less)" :
                    rssi < 10 ? "Weak" :
                    rssi < 20 ? "Moderate" :
                    rssi < 31 ? "Good" : "Excellent (-51 dBm or greater)");
            return rssi;
        } else if (rssi == 99) {
            LOG_WRN("  Signal not detectable");
            return 99;
        } else {
            LOG_WRN("  Invalid RSSI value: %d, returning 99", rssi);
            return 99;
        }
    } else {
        LOG_WRN("Failed to parse +CSQ response, returning 99 (unknown)");
        return 99;
    }
}


/* Retrieves the full 15-digit IMEI from the modem via AT+CGSN and stores
 * it as a NUL-terminated string in buf (must be at least 16 bytes). Snapshots
 * resp_buf under irq_lock before parsing to avoid a race with uart_cb(). */
int iridium_get_imei(char *buf, size_t buf_len)
{
    static const char cmd[] = "AT+CGSN\r";

    if (buf == NULL || buf_len < 16U) {
        LOG_ERR("iridium_get_imei(): invalid argument");
        return -EINVAL;
    }

    LOG_INF("Reading IMEI (AT+CGSN)...");

    iridium_result_t result = iridium_send_cmd_verify_response(cmd, "OK", 10);
    if (result != IRIDIUM_OK) {
        LOG_WRN("AT+CGSN failed (%s)", iridium_result_str(result));
        return -EIO;
    }

    /* Parse response - IMEI is a 15-digit number on its own line
     * Example response:
     *   300234065114290
     *   OK
     */

    /* Copy resp_buf under irq_lock before parsing to avoid a race with uart_cb(). */
    char local_buf[IRIDIUM_RESP_MAX_LEN];
    unsigned int key = irq_lock();
    memcpy(local_buf, resp_buf, sizeof(local_buf));
    irq_unlock(key);

    /* Find the first line (IMEI) - skip any leading whitespace/newlines */
    char *imei_start = local_buf;
    while (*imei_start && (*imei_start == '\r' || *imei_start == '\n' || *imei_start == ' ')) {
        imei_start++;
    }

    /* IMEI should be 15 digits */
    int i;
    for (i = 0; i < 15 && imei_start[i] >= '0' && imei_start[i] <= '9'; i++) {
        buf[i] = imei_start[i];
    }
    buf[i] = '\0';

    if (i != 15) {
        LOG_ERR("Invalid IMEI format (expected 15 digits, got %d)", i);
        return -EBADMSG;
    }

    LOG_INF("IMEI: %s", buf);

    return 0;
}


/* Loads a text message into the modem SBD TX buffer via AT+SBDWT and
 * initiates a satellite exchange to send it. */
iridium_result_t iridium_send_text(const char *msg, int timeout_s)
{
    if (msg == NULL || timeout_s <= 0) {
        LOG_ERR("iridium_send_text(): invalid argument");
        return IRIDIUM_BAD_ARG;
    }

    iridium_result_t result = modem_prepare();
    if (result != IRIDIUM_OK) {
        return result;
    }


    /* Build "AT+SBDWT=<msg>\r" */
    char cmd[IRIDIUM_CMD_MAX_LEN];
    int written = snprintf(cmd, sizeof(cmd), "AT+SBDWT=%s\r", msg);
    if (written < 0 || (size_t)written >= sizeof(cmd)) {
        LOG_ERR("iridium_send_text(): message too long");
        return IRIDIUM_BAD_ARG;
    }

    LOG_INF("Writing text to SBD TX buffer: \"%s\"", msg);

    result = iridium_send_cmd_verify_response(cmd, "OK", timeout_s);
    if (result != IRIDIUM_OK) {
        LOG_ERR("AT+SBDWT failed (%s)", iridium_result_str(result));
        return result;
    }

    LOG_INF("Initiating satellite exchange (AT+SBDIX)...");
    LOG_WRN("Satellite acquisition may take up to %d s", TIMEOUT_SBDIX_S);

    result = iridium_sbd_initiate_session(TIMEOUT_SBDIX_S);

    if (result == IRIDIUM_OK) {
        LOG_INF("Text message sent successfully");
    } else {
        LOG_ERR("Text message send failed (%s)", iridium_result_str(result));
    }

    return result;
}



/* -------------------------------------------------------------------------
 * Singleton struct instance
 * ---------------------------------------------------------------------- */

static const iridium_t satellite_instance = {
    .uart_init    = iridium_uart_init,
    .send_cmd     = iridium_send_cmd_verify_response,
    .sbd_write    = iridium_sbd_write_binary,
    .sbd_initiate = iridium_sbd_initiate_session,
    .get_rssi     = iridium_get_signal_quality,
    .get_imei     = iridium_get_imei,
    .result_str   = iridium_result_str,
    .send_text    = iridium_send_text,
    .send_binary  = iridium_send_binary_frame,
};

const iridium_t *iridium_get(void)
{
    return &satellite_instance;
}


/* -------------------------------------------------------------------------
 * Function summary
 *
 * ┌────────────────────────────────────┬────────────────────────────────────────────────────┐
 * │              Function              │                 Comment describes                  │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ uart_cb                            │ (already has inline comments inside)               │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ resp_buf_reset                     │ Clears accumulator under IRQ lock                  │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ uart_tx_and_wait_txdone            │ Sends raw bytes, waits for TX_DONE                 │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ sbd_checksum                       │ Simple 16-bit byte-sum checksum                    │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_uart_init                  │ Registers callback, enables RX, waits for boot     │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_send_cmd_verify_response   │ Sends AT command, blocks on semaphore for response │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_sbd_write_binary           │ Loads binary payload via AT+SBDWB with checksum    │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_sbd_initiate_session       │ Triggers satellite exchange via AT+SBDIX           │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_result_str                 │ Converts result code to readable string            │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_get_signal_quality         │ Queries RSSI via AT+CSQ                            │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_get_imei                   │ Retrieves 15-digit IMEI via AT+CGSN                │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ transmit_data_frame                │ Writes frame to modem and initiates exchange       │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_send_binary_frame          │ AT check then calls transmit_data_frame            │
 * ├────────────────────────────────────┼────────────────────────────────────────────────────┤
 * │ iridium_send_text                  │ Loads text via AT+SBDWT and initiates exchange     │
 * └────────────────────────────────────┴────────────────────────────────────────────────────┘
 * ---------------------------------------------------------------------- */