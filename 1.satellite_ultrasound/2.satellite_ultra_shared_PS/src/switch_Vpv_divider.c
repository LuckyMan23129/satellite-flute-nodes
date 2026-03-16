/**
 * @file switch_Vpv_divider.c
 * @brief Solar Panel Voltage Divider Control
 *
 * This module controls a digital switch (SW3) that enables/disables
 * a voltage divider circuit used to measure the solar panel voltage (Vpv).
 * The divider scales down the solar panel voltage to a range suitable
 * for ADC measurement.
 *
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "switch_Vpv_divider.h"

LOG_MODULE_REGISTER(switch_Vpv_divider);

// State tracking for Vpv divider status (atomic for thread-safe access)
static atomic_t vpv_divider_enabled = ATOMIC_INIT(0);

#define SW_DIV DT_ALIAS(vpvdividersw)
static const struct gpio_dt_spec sw_div = GPIO_DT_SPEC_GET(SW_DIV, gpios);

/**
 * @brief Initialize GPIO for Vpv voltage divider switch control
 *
 * Configures the GPIO pin for the digital switch that controls
 * the solar panel voltage divider. Must be called before using
 * enable_Vpv_divider() or disable_Vpv_divider().
 *
 * @return 0 on success, 1 on failure (GPIO not ready or configuration failed)
 */
int8_t check_gpio_Vpv_divider(void)
{
    if (!device_is_ready(sw_div.port)) {
        LOG_ERR("Vpv Divider - GPIO initialization for controlling the digital switch is not successful!");
        return (1);
    }
    int8_t ret_SW3 = gpio_pin_configure_dt(&sw_div, GPIO_OUTPUT_ACTIVE);
    if (ret_SW3 < 0) {
        LOG_ERR("Vpv Divider - Pin configuration for controlling the digital switch is not successful!");
        return (1);
    }
    return 0;
}

/**
 * @brief Enable the Vpv voltage divider
 *
 * Turns on the digital switch to enable the voltage divider circuit,
 * allowing measurement of the solar panel voltage. Should be enabled
 * before calling read_adc() for Vpv measurement.
 */
void enable_Vpv_divider(void)
{
    gpio_pin_set_dt(&sw_div, 1);     // Pin HIGH => Switch on
    atomic_set(&vpv_divider_enabled, 1);
}

/**
 * @brief Disable the Vpv voltage divider
 *
 * Turns off the digital switch to disable the voltage divider circuit.
 * Should be disabled after Vpv measurement to save power.
 */
void disable_Vpv_divider(void)
{
    gpio_pin_set_dt(&sw_div, 0);     // Pin LOW => Switch off
    atomic_set(&vpv_divider_enabled, 0);
}

/**
 * @brief Query current Vpv divider state
 *
 * Returns the current state of the voltage divider switch without
 * performing any hardware operations.
 *
 * @return true if voltage divider is enabled, false if disabled
 */
bool is_Vpv_divider_enabled(void)
{
    return (bool)atomic_get(&vpv_divider_enabled);
}