//=============================================================================#
/**
 * @brief: AsTAR++ Algorithm
 *
 * Energy-adaptive task scheduling algorithm for battery-free IoT devices.
 * All internal state is encapsulated - access through provided functions only.
 */
//=============================================================================#

#ifndef APPLICATION_ASTAR_DIURNAL_H
#define APPLICATION_ASTAR_DIURNAL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

// ---------------------------------------------------------------------------#
// Configuration Structure                                                    #
// ---------------------------------------------------------------------------#

typedef struct {
    // Voltage thresholds (mV)
    uint16_t maxVoltage;              // Maximum allowable capacitor voltage
    uint16_t shutOffVoltage;          // Voltage to suspend system (above brown-out)
    uint16_t daytimeOptimumV;         // Target voltage during daytime
    uint16_t OpenCircuitVoltage;      // Voltage to disconnect solar panel
    uint16_t wakeup_Vpv_Threshold;    // Vpv threshold to enter daytime mode
    uint16_t sleep_Vpv_Threshold;     // Vpv threshold to enter nighttime mode
    uint16_t check_overVcap_threshold;// Vcap threshold to check overvoltage

    // Timing parameters (seconds)
    uint32_t maxRate;                 // Minimum sleep interval (fastest rate)
    uint32_t minRate;                 // Maximum sleep interval (slowest rate)
    uint32_t nighttimeMaxRate;        // Minimum sleep interval at night
    uint16_t LowVolt_SleepTime;       // Sleep time when voltage is critical
    uint32_t daily_initial_wakeup_SleepTime;  // Sleep time at first daily wakeup
    uint32_t daily_initial_sleep_SleepTime;   // Sleep time at first daily sleep
    uint32_t nightDurationRollingEstimate;    // Initial EWMA night duration estimate

    // Hardware configuration
    bool USE_BOOST;                   // true if using boost converter
} astar_config_t;

// ---------------------------------------------------------------------------#
// Initialization Functions                                                   #
// ---------------------------------------------------------------------------#

/**
 * @brief Initialize AsTAR with provided configuration
 * @param config Pointer to configuration structure
 */
void astar_init(const astar_config_t *config);

// ---------------------------------------------------------------------------#
// Core Functions                                                             #
// ---------------------------------------------------------------------------#

/**
 * @brief Update the current capacitor voltage reading
 * @param vcap_mv Capacitor voltage in millivolts
 */
uint16_t astar_safe_read_vcap(void);
void astar_safe_update_vcap(uint16_t vcap_mv);

/**
 * @brief Check if system should suspend due to low voltage
 * @return true if Vcap <= shutOffVoltage
 */
bool astar_should_suspend(void);

/**
 * @brief Handle low-voltage suspension
 */
void setSuspensionHandler(void);

/**
 * @brief Run scheduler to compute next sleep interval
 * @return Recommended sleep duration in seconds
 */
uint32_t schedule(void);

/**
 * @brief Overvoltage protection thread function
 */
void overV_protection_thread(void);

// ---------------------------------------------------------------------------#
// Accessor Functions (read-only access to internal state)                    #
// ---------------------------------------------------------------------------#

// Runtime state accessors
uint16_t astar_get_vcap_mv(void);
uint16_t astar_get_vpv_mv(void);
uint32_t astar_get_sleep_timer(void);
bool astar_is_nighttime(void);
uint32_t astar_get_time_since_sunset(void);

// Configuration accessors
uint16_t astar_get_max_voltage(void);
uint16_t astar_get_shutoff_voltage(void);
uint16_t astar_get_open_circuit_voltage(void);
uint32_t astar_get_max_rate(void);
uint32_t astar_get_min_rate(void);
uint32_t astar_get_night_duration_estimate(void);
bool astar_get_use_boost(void);

// ---------------------------------------------------------------------------#
// Shared Synchronisation Primitives                                          #
// ---------------------------------------------------------------------------#
/* Serialises modem_transmitData() (main thread) against enable_charging() +
 * rail-stabilisation sleep (overV thread).  Defined in astar_diurnal.c.
 * Always acquire before calling modem_transmitData(). */
extern struct k_mutex modem_rail_mutex;

#endif  /*APPLICATION_ASTAR_DIURNAL_H*/
