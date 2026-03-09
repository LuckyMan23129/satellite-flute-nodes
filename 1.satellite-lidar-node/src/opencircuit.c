/**
 * @file opencircuit.c
 * @brief Solar Panel Open-Circuit Control
 *
 * This module controls a digital switch that connects or disconnects
 * the solar panel from the storage capacitor. Used for overvoltage protection
 * and open-circuit voltage measurement.
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
#include "opencircuit.h"

LOG_MODULE_REGISTER(opencircuit);

// State tracking for charging status (atomic for thread-safe access)
static atomic_t charging_enabled = ATOMIC_INIT(1);

static int8_t ret_open;
#define open_circuit DT_ALIAS(opencircuit)
static const struct gpio_dt_spec opencircuit_sw = GPIO_DT_SPEC_GET(open_circuit, gpios);

/**
 * @brief Initialize GPIO for open-circuit switch control
 *
 * Configures the GPIO pin for the digital switch that controls
 * solar panel connection. Must be called before using enable_charging()
 * or disable_charging().
 *
 * @return 0 on success, 1 on failure (GPIO not ready or configuration failed)
 */

int8_t check_gpio_sw_opencircuit(void)
{
    if (!device_is_ready(opencircuit_sw.port)) {
        LOG_ERR("Opencircuit - GPIO initialization for controlling the digital switch is not successful!");
        return (1);
    }
    ret_open = gpio_pin_configure_dt(&opencircuit_sw, GPIO_OUTPUT_ACTIVE);
    if (ret_open < 0) {
        LOG_ERR("Opencircuit - Pin configuration for controlling the digital switch is not successful!");
        return (1);
    }
    return 0;
}

/**
 * @brief Enable solar panel charging
 *
 * Turns on the digital switch to connect the solar panel to the
 * storage capacitor, allowing charging to occur.
 */
void enable_charging(void)
{
    gpio_pin_set_dt(&opencircuit_sw, 1);        // Pin HIGH => Switch on
    atomic_set(&charging_enabled, 1);
}

/**
 * @brief Disable solar panel charging
 *
 * Turns off the digital switch to isolate the solar panel from
 * the storage capacitor. Used for overvoltage protection and
 * open-circuit voltage measurement.
 */
void disable_charging(void)
{
    gpio_pin_set_dt(&opencircuit_sw, 0);        // Pin LOW => Switch off
    atomic_set(&charging_enabled, 0);
}

/**
 * @brief Query current charging state
 *
 * Returns the current state of the charging switch without
 * performing any hardware operations.
 *
 * @return true if charging is enabled, false if disabled
 */
bool is_charging_enabled(void)
{
    return (bool)atomic_get(&charging_enabled);
}
