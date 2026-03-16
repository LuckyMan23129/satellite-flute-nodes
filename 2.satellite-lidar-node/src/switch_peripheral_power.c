/**
 * @file switch_peripheral_power.c
 * @brief Peripheral Power Switch Control
 *
 * This module controls a digital switch that powers on/off peripherals
 * (e.g. sensors). Peripherals are powered down when not in use to save
 * energy.
 *
 *
 * @author DistriNet LAB, KU Leuven
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>


#include "switch_peripheral_power.h"
#include "config.h"

static int8_t ret;

LOG_MODULE_REGISTER(switch_peripheral_power);

#define PERI_SW DT_ALIAS(periphsw)

// Get the device pointer, pin number, and pin's configuration flags through gpio_dt_spec
static const struct gpio_dt_spec perip_sw  = GPIO_DT_SPEC_GET(PERI_SW, gpios);

/**
 * @brief Initialize GPIO for peripheral power switch control
 *
 * Configures the GPIO pin for the digital switch that controls
 * peripheral power. Must be called before using periph_sw_turn_on()
 * or periph_sw_turn_off().
 *
 * @return 0 on success, 1 on failure (GPIO not ready or configuration failed)
 */
int8_t check_gpio_periph_sw(void){
    if (!device_is_ready(perip_sw.port)) {
        LOG_ERR("GPIO initialization for Digital SW is not successful!");
        return (1);
    }

    // ret = gpio_pin_configure_dt(&perip_sw, GPIO_PULL_UP | GPIO_OUTPUT_ACTIVE);
    ret = gpio_pin_configure_dt(&perip_sw, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Pin Configuration for Digital SW is not successful!");
        return (1);
    }
    return 0;
}

/**
 * @brief Turn on peripheral power switch
 *
 * Turns on the digital switch to power the peripherals (e.g. sensors)
 * so that data can be read from them.
 */
void periph_sw_turn_on(void){
    gpio_pin_set_dt(&perip_sw,1);
}

/**
 * @brief Turn off peripheral power switch
 *
 * Turns off the digital switch to power down the peripherals
 * (e.g. sensors) to save energy when not in use.
 */
void periph_sw_turn_off(void){
    gpio_pin_set_dt(&perip_sw,0);
}