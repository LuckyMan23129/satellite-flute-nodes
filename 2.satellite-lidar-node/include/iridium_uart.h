/*
 * iridium_uart.h — UART transport layer for Iridium 9602/9603 modem
 *
 * SEMAPHORE VERSION — Event-driven approach with zero polling overhead.
 *
 * Uses semaphores to block threads until responses arrive, eliminating
 * the need for periodic polling loops. The UART callback signals the
 * semaphore immediately when a complete response is detected.
 *
 * Hardware: nRF9160, Iridium module on uart2 (19 200 baud, set in overlay).
 */

#ifndef IRIDIUM_UART_H
#define IRIDIUM_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Application configuration
 * ---------------------------------------------------------------------- */

/** Pause between transmission cycles (milliseconds). */
#define CYCLE_INTERVAL_MS    300000

/** AT basic command timeout (seconds). */
#define TIMEOUT_AT_S         60

/** AT+SBDWB timeout (seconds). */
#define TIMEOUT_SBDWB_S      10

/** AT+SBDIX timeout (seconds) — satellite acquisition can be slow. */
#define TIMEOUT_SBDIX_S      120

/** Sleep time reported in the data frame (seconds). */
#define SLEEP_TIME           300U

/** Vcap voltage reported in the data frame (mV). */
#define VCAP                 2900U

/** Vpv voltage reported in the data frame (mV). */
#define VPV                  5000U

/** Data frame size (bytes). */
#define FRAME_SIZE           10U

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

/** Maximum AT command length including the trailing CR (bytes). */
#define IRIDIUM_CMD_MAX_LEN          128U

/** Maximum accumulated response length (bytes). */
#define IRIDIUM_RESP_MAX_LEN         512U

/**
 * Maximum binary SBD payload (data only, excluding the 2-byte checksum).
 * The Iridium SBD protocol supports up to 340 bytes.
 */
#define IRIDIUM_SBD_MAX_BINARY_LEN   340U

/* -------------------------------------------------------------------------
 * Return / result codes
 * ---------------------------------------------------------------------- */

/**
 * @brief  Result of any Iridium modem operation.
 */
typedef enum {
    IRIDIUM_OK       =  0,      /**< Operation succeeded.                   */
    IRIDIUM_ERROR    = -1,      /**< Modem returned ERROR.                  */
    IRIDIUM_TIMEOUT  = -2,      /**< No response within the timeout.        */
    IRIDIUM_TX_FAIL  = -3,      /**< Failed to transmit over UART.          */
    IRIDIUM_INIT_ERR = -4,      /**< Driver initialisation failure.         */
    IRIDIUM_BAD_ARG  = -5,      /**< Invalid argument (NULL / out-of-range).*/
    IRIDIUM_CHKSUM      = -6,   /**< Modem rejected binary checksum.        */
    IRIDIUM_NO_NETWORK  = -7,   /**< SBDIX MO status: no network service.   */
} iridium_result_t;

/* -------------------------------------------------------------------------
 * Struct API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Function pointer table for the Iridium modem interface.
 *
 * Obtain a pointer to the singleton instance via iridium_get().
 * Call members with the -> operator:
 *
 *   const iridium_t *satellite = iridium_get();
 *   satellite->uart_init();
 *   satellite->send_text("hello", 60);
 */
typedef struct {
    /** Initialise the UART transport (call once at startup). */
    int              (*uart_init)    (void);

    /** Send an AT command and block until expected response or timeout. */
    iridium_result_t (*send_cmd)     (const char *cmd,
                                      const char *expected_resp,
                                      int         timeout_s);

    /** Load binary payload into SBD TX buffer via AT+SBDWB. */
    iridium_result_t (*sbd_write)    (const uint8_t *data,
                                      size_t         len,
                                      int            timeout_s);

    /** Initiate satellite SBD exchange via AT+SBDIX. */
    iridium_result_t (*sbd_initiate) (int timeout_s);

    /** Query RSSI via AT+CSQ. Returns 0-31, or 99 if unknown. */
    int              (*get_rssi)     (void);

    /** Retrieve 15-digit IMEI via AT+CGSN into buf (>= 16 bytes). */
    int              (*get_imei)     (char *buf, size_t buf_len);

    /** Convert a result code to a human-readable string. */
    const char      *(*result_str)   (iridium_result_t result);

    /** Load text via AT+SBDWT and initiate satellite exchange. */
    iridium_result_t (*send_text)    (const char *msg, int timeout_s);

    /** AT check then write frame to modem and initiate exchange. */
    void             (*send_binary)  (const uint8_t *frame, size_t len);
} iridium_t;

/**
 * @brief  Return a pointer to the singleton Iridium interface.
 *
 * Usage:
 *   const iridium_t *satellite = iridium_get();
 *   satellite->uart_init();
 *
 * @return  Pointer to the static iridium_t instance; never NULL.
 */
const iridium_t *iridium_get(void);

#ifdef __cplusplus
}
#endif

#endif /* IRIDIUM_UART_H */
