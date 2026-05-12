/**
 * @file astar_diurnal.c
 * @brief AsTAR++ Algorithm Implementation
 *
 * This file implements the AsTAR++ energy-adaptive task scheduling algorithm
 * for battery-free IoT devices. It manages energy harvesting and consumption
 * by dynamically adjusting task execution rates based on capacitor voltage
 * and solar panel conditions.
 *
 * @author DistriNet LAB, KU Leuven
 *
 * MIT License
 *
 * Copyright (c) 2026 DistriNet, KU Leuven
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "astar_diurnal.h"
#include "config.h"
#include "read_Vcap.h"
#include "read_Vsupply.h"
#include "read_Vpv.h"
#include "opencircuit.h"

LOG_MODULE_REGISTER(AsTAR);

// ---------------------------------------------------------------------------
// Internal State Structure (private)
// ---------------------------------------------------------------------------

typedef struct {
    // Current measurements
    uint16_t newV;                    // Current capacitor voltage (mV)
    uint16_t solarV;                  // Current solar panel voltage (mV)
    uint16_t oldV;                    // Previous capacitor voltage (mV)
    int16_t  deltaV;                  // Voltage change since last cycle

    // Scheduling state
    uint32_t sleepTimer;              // Current sleep interval (seconds)
    uint32_t optimumV;                // Current target voltage

    // Night tracking
    bool     nighttimeFlag;           // true if in nighttime mode
    uint32_t timeSinceSunset;         // Seconds since sunset detected
    uint32_t timeSinceSunrise;        // Seconds since sunrise detected
    uint32_t nightDurationRollingEstimate;  // EWMA night duration estimate
    uint16_t beginSleeping_Vcap;      // Vcap when entering nighttime
    uint16_t nighttimeVSwing;         // Voltage swing during night

    // Daily transition flags
    bool     daily_first_wakeup_flag;
    bool     daily_first_sleep_flag;
} astar_state_t;

// ---------------------------------------------------------------------------
// Static Internal Instances (encapsulated)
// ---------------------------------------------------------------------------

static astar_config_t config;
static astar_state_t  state;
static bool           initialized = false;


// ---------------------------------------------------------------------------#
// Power-Rail Protection                                                      #
// ---------------------------------------------------------------------------#
/* Protects all reads/writes of the astar_state_t struct across threads.
 * Lock order when modem_rail_mutex is also needed:
 *   vcap_mutex → modem_rail_mutex  (never reversed)
 * Lock order when adc_mutex is also needed:
 *   vcap_mutex → adc_mutex         (never reversed) */
K_MUTEX_DEFINE(vcap_mutex);

/**
 * @brief Shared mutex that serialises modem_transmitData() (main thread) against
 *        enable_charging() + rail-stabilisation sleep (overV thread), preventing
 *        solar panel reconnection from disrupting an in-progress transmission.
 *        Declared extern in astar_diurnal.h so main.c can acquire it.
 *        Lock order when vcap_mutex is also needed:
 *          vcap_mutex → modem_rail_mutex  (never reversed)
 */
K_MUTEX_DEFINE(modem_rail_mutex);

/* Serialises all SAADC accesses across threads (overV thread, main, schedule). */
K_MUTEX_DEFINE(adc_mutex);


// Internal constants for day/night transition detection
static const uint16_t nightVLossTimeThreshold = 300;  // Time threshold to confirm sunset (seconds)
static const uint16_t nightVRiseTimeThreshold = 300;  // Time threshold to confirm sunrise (seconds)
static const uint8_t  weightingNewNightLength = 30;   // EWMA weighting for new night length (%)

// ---------------------------------------------------------------------------
// Initialization Functions
// ---------------------------------------------------------------------------

/**
 * @brief Initialize the AsTAR algorithm with provided configuration
 *
 * This function copies the provided configuration and initializes the runtime
 * state to default values.
 *
 * @param cfg Pointer to the configuration structure containing all AsTAR parameters
 */
void astar_init(const astar_config_t *cfg)
{
    // Copy configuration
    config = *cfg;

    // Initialize runtime state
    state.newV = 0;
    state.solarV = 0;
    state.oldV = 0;
    state.deltaV = 0;
    state.sleepTimer = 0;
    state.optimumV = cfg->daytimeOptimumV;
    state.nighttimeFlag = false;
    state.timeSinceSunset = 0;
    state.timeSinceSunrise = 0;
    state.nightDurationRollingEstimate = cfg->nightDurationRollingEstimate;
    state.beginSleeping_Vcap = 0;
    state.nighttimeVSwing = 0;
    state.daily_first_wakeup_flag = false;
    state.daily_first_sleep_flag = false;

    initialized = true;
}

// ---------------------------------------------------------------------------
// Core Functions
// ---------------------------------------------------------------------------

/**
 * @brief Update the current capacitor voltage in the internal state
 *
 * This function provides thread-safe update of the capacitor voltage.
 * It uses vcap_mutex to prevent race conditions with the overvoltage
 * protection thread.
 *
 * @param vcap_mv Current capacitor voltage in millivolts
 */
void astar_safe_update_vcap(uint16_t vcap_mv)
{
    if (!initialized) return;
    k_mutex_lock(&vcap_mutex, K_FOREVER);
    state.newV = vcap_mv;
    k_mutex_unlock(&vcap_mutex);
}

/**
 * @brief Read capacitor voltage from ADC
 * 
 * @return Capacitor voltage in millivolts
 */
uint16_t astar_safe_read_vcap(void)
{
    if (!initialized) return 0;
    k_mutex_lock(&adc_mutex, K_FOREVER);
    uint16_t v = config.USE_BOOST ? read_Vcap_mv() : read_Vsupp_mv();
    k_mutex_unlock(&adc_mutex);
    return v;
}

/**
 * @brief Check if the system should suspend due to low voltage
 *
 * Compares the current capacitor voltage against the configured
 * shutoff threshold to determine if the system should enter
 * low-power suspension mode.
 *
 * @return true if Vcap <= shutOffVoltage, false otherwise
 */
bool astar_should_suspend(void)
{
    return (state.newV <= config.shutOffVoltage);
}


// ---------------------------------------------------------------------------
// Overvoltage Protection Thread
// ---------------------------------------------------------------------------

/**
 * @brief Background thread for capacitor overvoltage protection
 *
 * This thread runs continuously in the background, periodically checking
 * the capacitor voltage to prevent overvoltage conditions. When voltage
 * exceeds the OpenCircuitVoltage threshold, it disconnects the solar
 * panel to stop charging.
 *
 * Check frequency:
 * - Daytime: every 60 seconds
 * - Nighttime: every 1800 seconds (30 minutes) to save energy
 */
void overV_protection_thread(void)
{
    while (1)
    {
        if (ENABLE_PRINT)
            LOG_INF("+++++ Entered OverVcap Protection Thread +++++");

        /* Step 1: hold mutex only long enough to read flags/thresholds — no ADC here. */
        bool nighttime = false;
        bool do_adc    = false;
        if (k_mutex_lock(&vcap_mutex, K_SECONDS(5)) == 0)
        {
            // Only check during daytime when voltage is high enough to matter
            do_adc   = (!state.nighttimeFlag) && (state.oldV > config.check_overVcap_threshold);
            nighttime = state.nighttimeFlag;
            k_mutex_unlock(&vcap_mutex);
        }
        else
        {
            if (ENABLE_PRINT)
                LOG_INF("Thread timed out waiting for vcap_mutex");
        }

        /* Step 2: ADC read happens outside the semaphore — takes ~100 ms but
         * does NOT block schedule() any more.  Then re-take the semaphore for
         * the brief state write and GPIO action. */
        if (do_adc)
        {
            k_mutex_lock(&adc_mutex, K_FOREVER);
            uint16_t  v = config.USE_BOOST ? read_Vcap_mv() : read_Vsupp_mv();
            k_mutex_unlock(&adc_mutex);
            bool     did_enable    = false;
            uint16_t solar_snapshot = 0;

            if (k_mutex_lock(&vcap_mutex, K_SECONDS(5)) == 0)
            {
                if (v > config.OpenCircuitVoltage) {
                    disable_charging();
                } else {
                    /* Acquire modem_rail_mutex INSIDE vcap_mutex so that
                     * enable_charging() and the rail-stabilisation sleep form
                     * an atomic unit and the main thread cannot interleave a
                     * modem_transmitData() call between them.
                     * Lock order: vcap_mutex → modem_rail_mutex
                     * (never reversed). */
                    k_mutex_lock(&modem_rail_mutex, K_FOREVER);
                    enable_charging();
                    did_enable    = true;
                    solar_snapshot = state.solarV;  // capture under lock to avoid data race
                }
                k_mutex_unlock(&vcap_mutex);
            }

            /* Rail stabilisation: sleep outside vcap_mutex but still
             * inside modem_rail_mutex so modem TX waits until the inrush
             * current from the reconnected solar panel has settled. */
            if (did_enable) {
                if ((v > config.daytimeOptimumV) && (solar_snapshot > (v + 800U)))
                    k_sleep(K_MSEC(100));
                k_mutex_unlock(&modem_rail_mutex);
            }
        }

        // Check less frequently at night to save energy
        if (nighttime)
            k_sleep(K_SECONDS(1800));
        else
            k_sleep(K_SECONDS(60));

        if (ENABLE_PRINT)
            LOG_INF("+++++++ Escaped OverVcap Protection Thread ++++++++");
    }
}

// ---------------------------------------------------------------------------
// Scheduler Functions
// ---------------------------------------------------------------------------

/**
 * @brief Handle low-voltage suspension
 *
 * Called when the capacitor voltage drops below the shutoff threshold.
 * Puts the MCU into deep sleep for the configured low-voltage sleep time
 * to allow the capacitor to recharge.
 */
void setSuspensionHandler(void)
{
    if (ENABLE_PRINT)
        LOG_INF("Vcap is very low, so the MCU enters sleep mode immediately for %d (s)",
                config.LowVolt_SleepTime);

    k_mutex_lock(&vcap_mutex, K_FOREVER);
    state.sleepTimer = config.LowVolt_SleepTime;
    state.oldV = state.newV;
    k_mutex_unlock(&vcap_mutex);

    k_sleep(K_SECONDS(config.LowVolt_SleepTime));
}

/**
 * @brief Main AsTAR++ scheduling function
 *
 * This is the core scheduling algorithm that determines the optimal sleep
 * interval based on current energy conditions. It implements:
 *
 * 1. Day/Night Detection: Uses solar panel voltage (Vpv) with hysteresis
 *    to detect transitions between daytime and nighttime modes.
 *
 * 2. Daytime Mode: Maintains capacitor voltage near daytimeOptimumV using
 *    proportional control to adjust sleep intervals.
 *
 * 3. Nighttime Mode: Gradually drains ~90% of available charge by estimated
 *    sunrise using EWMA-based night duration prediction.
 *
 * 4. Overvoltage Protection: Disconnects solar panel when voltage too high.
 *
 * @return Recommended sleep duration in seconds
 */
uint32_t schedule(void)
{
    uint32_t sleepDelta;

    // Hold state lock for entire scheduling computation to prevent races with overV thread
    k_mutex_lock(&vcap_mutex, K_FOREVER);

    /** state.newV is already set by astar_safe_update_vcap() in main.c before
     *  schedule() is called. Re-reading here is unnecessary and waste energy
     *  So, we comment this
    */
    // if (config.USE_BOOST)
    //     state.newV = read_Vcap_mv();
    // else
    //     state.newV = read_Vsupp_mv();

    if (ENABLE_PRINT)
        LOG_INF("The supercapacitor Voltage - Vcap = %d mV", state.newV);

    
    // Update solar panel voltage
    disable_charging();  // Isolate the solar panels from Capacitors to measure open-circuit Vpv
    k_mutex_lock(&adc_mutex, K_FOREVER);
    state.solarV = update_Vpv();
    k_mutex_unlock(&adc_mutex);
    
    // DO this after sending data to avoid the inrush current issue when reconnecting solar panel at high Vpv 
    // Reconnect solar only when Vcap is below the opencircuit threshold because we disable it before reading Vpv
    // if (state.newV > config.OpenCircuitVoltage)
    //     disable_charging();
    // else
    //     enable_charging();


    // Calculate voltage change since last cycle
    state.deltaV = state.newV - state.oldV;

    // --- Day/Night Transition Detection ---

    // Check for sunrise (transition from night to day)
    if (state.nighttimeFlag)
    {
        if (state.solarV >= config.wakeup_Vpv_Threshold)
            state.timeSinceSunrise += state.sleepTimer;
        else
            state.timeSinceSunrise = 0;

        // Confirm sunrise after threshold time
        if (state.timeSinceSunrise >= nightVRiseTimeThreshold)
        {
            // Update EWMA night duration estimate (only for nights > 4 hours)
            if (state.timeSinceSunset >= 14400)
            {
                state.nightDurationRollingEstimate =
                    (((100 - weightingNewNightLength) * state.nightDurationRollingEstimate) / 100) +
                    ((weightingNewNightLength * state.timeSinceSunset) / 100);
            }
            state.nighttimeFlag = false;
            state.optimumV = config.daytimeOptimumV;
            state.timeSinceSunset = 0;
            state.daily_first_wakeup_flag = true;
        }
    }

    
    // Check for sunset (transition from day to night)
    if (state.solarV < config.sleep_Vpv_Threshold)
    {
        state.timeSinceSunset += state.sleepTimer;

        // Confirm sunset after threshold time
        if (state.timeSinceSunset >= nightVLossTimeThreshold)
        {
            if (!state.nighttimeFlag)
            {
                // Record voltage at start of night for discharge planning
                state.beginSleeping_Vcap = state.newV;
                uint32_t safety_floor = (config.shutOffVoltage * 11U) / 10U;
                state.nighttimeVSwing = (state.beginSleeping_Vcap > safety_floor)
                                        ? (state.beginSleeping_Vcap - safety_floor) : 0U;
                state.daily_first_sleep_flag = true;
            }
            state.nighttimeFlag = true;
        }
    }



    // --- Calculate Target Voltage (optimumV) ---

    if (state.nighttimeFlag)
    {
        // Nighttime: gradually decrease target voltage to drain 90% by sunrise
        state.optimumV = state.beginSleeping_Vcap -
            ((state.nighttimeVSwing * state.timeSinceSunset) / state.nightDurationRollingEstimate);

        // Ensure target doesn't go below safety margin
        uint32_t safety_floor_night = (config.shutOffVoltage * 11U) / 10U;
        if (state.optimumV < safety_floor_night)
            state.optimumV = safety_floor_night;
    }

    // --- Calculate Proportional Control Gains ---

    float kp1, kp2;
    kp1 = (state.newV - state.optimumV) / 50.0f;
    if (kp1 < 1.5f) kp1 = 1.5f;
    if (kp1 > 4.0f) kp1 = 4.0f;

    kp2 = (state.optimumV - state.newV) / 50.0f;
    if (kp2 < 1.5f) kp2 = 1.5f;
    if (kp2 > 4.0f) kp2 = 4.0f;

    // --- Determine Sleep Timer Based on Voltage State ---

    if (state.newV >= config.maxVoltage)
    {
        // Voltage at maximum: use fastest rate to consume energy
        state.sleepTimer = config.maxRate;
    }
    else if ((state.newV < config.maxVoltage) && (state.newV > state.optimumV))
    {
        // Voltage above target: adjust rate to reduce voltage
        if (state.deltaV >= 0)
            state.sleepTimer = state.sleepTimer / kp1;

        if (state.deltaV < 10)
        {
            sleepDelta = state.sleepTimer / 10;
            if (sleepDelta < 1) sleepDelta = 1;

            if (state.nighttimeFlag)
                state.sleepTimer -= sleepDelta;  // Night: speed up to drain
            else
                state.sleepTimer += sleepDelta;  // Day: slow down
        }

        // Apply rate limits
        if (state.sleepTimer < config.maxRate)
            state.sleepTimer = config.maxRate;
        if (state.nighttimeFlag && (state.sleepTimer < config.nighttimeMaxRate))
            state.sleepTimer = config.nighttimeMaxRate;
        if (state.sleepTimer > config.minRate)
            state.sleepTimer = config.minRate;
    }
    else if (state.newV < state.optimumV)
    {
        // Voltage below target: slow down to allow charging
        if (state.deltaV > 0)
        {
            sleepDelta = state.sleepTimer / 10;
            if (sleepDelta < 1) sleepDelta = 1;
            state.sleepTimer -= sleepDelta;
        }
        if (state.deltaV <= 0)
            state.sleepTimer = state.sleepTimer * kp2;

        // Apply rate limits
        if (state.sleepTimer < config.maxRate)
            state.sleepTimer = config.maxRate;
        if (state.nighttimeFlag && (state.sleepTimer < config.nighttimeMaxRate))
            state.sleepTimer = config.nighttimeMaxRate;
        if (state.sleepTimer > config.minRate)
            state.sleepTimer = config.minRate;
    }
    // When newV = optimumV => do nothing (keep current rate)

    // --- Handle Daily Transitions with Fixed Initial Intervals ---

    if (state.daily_first_wakeup_flag)
    {
        state.sleepTimer = config.daily_initial_wakeup_SleepTime;
        state.daily_first_wakeup_flag = false;
    }

    if (state.daily_first_sleep_flag)
    {
        state.sleepTimer = config.daily_initial_sleep_SleepTime;
        state.daily_first_sleep_flag = false;
    }

    // Save current voltage for next cycle's delta calculation
    state.oldV = state.newV;

    k_mutex_unlock(&vcap_mutex);  // release after all state reads/writes are complete

    if (ENABLE_PRINT)
        LOG_INF("Finished run the AsTAR scheduler - Sleeptimer = %d (s) ", state.sleepTimer);

    return state.sleepTimer;
}

// ---------------------------------------------------------------------------
// Accessor Functions Implementation
// ---------------------------------------------------------------------------

/**
 * @brief Get current capacitor voltage
 * @return Capacitor voltage in millivolts
 */
uint16_t astar_get_vcap_mv(void) {
    return state.newV;
}

/**
 * @brief Get current solar panel voltage
 * @return Solar panel voltage in millivolts
 */
uint16_t astar_get_vpv_mv(void) {
    return state.solarV;
}

/**
 * @brief Get current sleep timer value
 * @return Current sleep interval in seconds
 */
uint32_t astar_get_sleep_timer(void) {
    return state.sleepTimer;
}

/**
 * @brief Check if currently in nighttime mode
 * @return true if nighttime mode is active, false otherwise
 */
bool astar_is_nighttime(void) {
    return state.nighttimeFlag;
}

/**
 * @brief Get time elapsed since sunset was detected
 * @return Time since sunset in seconds
 */
uint32_t astar_get_time_since_sunset(void) {
    return state.timeSinceSunset;
}

/**
 * @brief Get configured maximum voltage threshold
 * @return Maximum voltage in millivolts
 */
uint16_t astar_get_max_voltage(void) {
    return config.maxVoltage;
}

/**
 * @brief Get configured shutoff voltage threshold
 * @return Shutoff voltage in millivolts
 */
uint16_t astar_get_shutoff_voltage(void) {
    return config.shutOffVoltage;
}

/**
 * @brief Get configured open circuit voltage threshold
 * @return Open circuit voltage in millivolts
 */
uint16_t astar_get_open_circuit_voltage(void) {
    return config.OpenCircuitVoltage;
}

/**
 * @brief Get configured maximum task rate (minimum sleep interval)
 * @return Maximum rate in seconds
 */
uint32_t astar_get_max_rate(void) {
    return config.maxRate;
}

/**
 * @brief Get configured minimum task rate (maximum sleep interval)
 * @return Minimum rate in seconds
 */
uint32_t astar_get_min_rate(void) {
    return config.minRate;
}

/**
 * @brief Get current EWMA night duration estimate
 * @return Estimated night duration in seconds
 */
uint32_t astar_get_night_duration_estimate(void) {
    return state.nightDurationRollingEstimate;
}

/**
 * @brief Check if boost converter is configured
 * @return true if boost converter is enabled, false otherwise
 */
bool astar_get_use_boost(void) {
    return config.USE_BOOST;
}
