/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_LOWPOWER_H_
#define APPLICATION_LOWPOWER_H_

/**
 * @file lowpower.h
 *
 * @brief Header file for low power management functions and configurations
 *
 * This file contains function prototypes and definitions related to low power
 * management, including GPIO setup and peripheral control for nrf9160.
 *
 * The low power management includes:
 * - Configuring GPIOs for power path control
 * - Disabling unused peripherals to reduce power consumption
 *
*/

/* Gpios */
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec latch_en = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), latch_en_gpios);

static const struct gpio_dt_spec wp = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), wp_gpios);
static const struct gpio_dt_spec hold = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), hold_gpios);


void lowpower_setup_accel(void);
int8_t lowpower_setup_gpio(void);
int8_t lowpower_setup_uart0_DIS(void);      // Disable UART0
int8_t lowpower_setup_uart0_ENA(void);      // Enable UART0
int8_t lowpower_setup_uart2_DIS(void);      // Disable UART2
int8_t lowpower_setup_uart2_ENA(void);      // Enable UART2

#endif /* APPLICATION_LOWPOWER_H_ */