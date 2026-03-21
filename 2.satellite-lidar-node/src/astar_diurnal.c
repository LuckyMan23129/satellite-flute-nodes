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
static struct k_sem   vcap_semaphore;
static bool           initialized = false;

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
 * This function copies the provided configuration, initializes the runtime
 * state to default values, and sets up the semaphore for thread-safe
 * voltage readings.
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
    state.optimumV = 0;
    state.nighttimeFlag = false;
    state.timeSinceSunset = 0;
    state.timeSinceSunrise = 0;
    state.nightDurationRollingEstimate = cfg->nightDurationRollingEstimate;
    state.beginSleeping_Vcap = 0;
    state.nighttimeVSwing = 0;
    state.daily_first_wakeup_flag = false;
    state.daily_first_sleep_flag = false;

    // Initialize semaphore for thread-safe ADC access
    k_sem_init(&vcap_semaphore, 1, 1);
    initialized = true;
}

// ---------------------------------------------------------------------------
// Core Functions
// ---------------------------------------------------------------------------

/**
 * @brief Update the current capacitor voltage in the internal state
 *
 * This function provides thread-safe update of the capacitor voltage.
 * It uses a semaphore to prevent race conditions with the overvoltage
 * protection thread.
 *
 * @param vcap_mv Current capacitor voltage in millivolts
 */
void astar_safe_update_vcap(uint16_t vcap_mv)
{
    if (!initialized) return;
    k_sem_take(&vcap_semaphore, K_FOREVER);
    state.newV = vcap_mv;
    k_sem_give(&vcap_semaphore);
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

        /* Step 1: hold semaphore only long enough to read flags/thresholds — no ADC here. */
        bool nighttime = false;
        bool do_adc    = false;
        if (k_sem_take(&vcap_semaphore, K_SECONDS(5)) == 0)
        {
            // Only check during daytime when voltage is high enough to matter
            do_adc   = (!state.nighttimeFlag) && (state.oldV > config.check_overVcap_threshold);
            nighttime = state.nighttimeFlag;
            k_sem_give(&vcap_semaphore);
        }
        else
        {
            if (ENABLE_PRINT)
                LOG_INF("Thread timed out waiting for vcap_semaphore");
        }

        /* Step 2: ADC read happens outside the semaphore — takes ~100 ms but
         * does NOT block schedule() any more.  Then re-take the semaphore for
         * the brief state write and GPIO action. */
        if (do_adc)
        {
            uint16_t v = config.USE_BOOST ? read_Vcap_mv() : read_Vsupp_mv();

            if (k_sem_take(&vcap_semaphore, K_SECONDS(5)) == 0)
            {
                state.newV = v;
                if (state.newV > config.OpenCircuitVoltage)
                    disable_charging();
                else
                    enable_charging();
                k_sem_give(&vcap_semaphore);
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

    state.sleepTimer = config.LowVolt_SleepTime;
    state.oldV = state.newV;
    k_sleep(K_SECONDS(state.sleepTimer));
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

    // Update solar panel voltage (solarV is not accessed by overV thread - no semaphore needed)
    state.solarV = update_Vpv();

    // Hold state lock for entire scheduling computation to prevent races with overV thread
    k_sem_take(&vcap_semaphore, K_FOREVER);

    // Read FRESH Vcap first -- update_Vpv() left charging disabled to avoid
    // an inrush current spike (Vsolar >> Vcap when Vsolar > 6000 mV).
    // We must know the actual current Vcap before deciding whether it is safe
    // to reconnect the solar panel.
    if (config.USE_BOOST)
        state.newV = read_Vcap_mv();
    else
        state.newV = read_Vsupp_mv();

    // Reconnect solar only when Vcap is below the maximum threshold.
    // This prevents the large inrush that occurs when Vsolar >> Vcap and that
    // was causing the modem to lose its LTE connection.
    if (state.newV > config.maxVoltage)
        disable_charging();
    else
        enable_charging();

    if (ENABLE_PRINT)
        LOG_INF("The supercapacitor Voltage - Vcap = %d mV", state.newV);

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
    if (kp1 > 5.0f) kp1 = 4.0f;

    kp2 = (state.optimumV - state.newV) / 50.0f;
    if (kp2 < 1.5f) kp2 = 1.5f;
    if (kp2 > 5.0f) kp2 = 4.0f;

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

    k_sem_give(&vcap_semaphore);  // release after all state reads/writes are complete

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
