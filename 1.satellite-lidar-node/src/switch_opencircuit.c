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
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "switch_opencircuit.h"

LOG_MODULE_REGISTER(opencircuit);

// State tracking for charging status
static bool charging_enabled = true;

static int8_t ret_open;
#define sw_open_circuit DT_ALIAS(opencircuit)
static const struct gpio_dt_spec sw_OpenCircuit = GPIO_DT_SPEC_GET(sw_open_circuit, gpios);

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
    if (!device_is_ready(sw_OpenCircuit.port)) {
        LOG_ERR("Opencircuit - GPIO initialization for controlling the digital switch is not successful!");
        return (1);
    }
    ret_open = gpio_pin_configure_dt(&sw_OpenCircuit, GPIO_OUTPUT_ACTIVE);
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
    gpio_pin_set_dt(&sw_OpenCircuit, 1);        // Pin HIGH => Switch on
    charging_enabled = true;
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
    gpio_pin_set_dt(&sw_OpenCircuit, 0);        // Pin LOW => Switch off
    charging_enabled = false;
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
    return charging_enabled;
}