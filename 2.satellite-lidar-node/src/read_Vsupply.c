/**
 * @file read_Vsupply.c
 * @brief MCU Supply Voltage Measurement
 *
 * This module provides ADC-based measurement of the MCU supply voltage (Vsupp).
 * Used when the system operates without a boost converter, measuring the voltage
 * that directly powers the microcontroller.
 *
 * 
 * @note Based on Nordic Semiconductor battery measurement sample code
 *
 * @copyright Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * @copyright Copyright (c) 2019-2020 Nordic Semiconductor ASA
 * @license SPDX-License-Identifier: Apache-2.0
 *
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "read_Vsupply.h"

LOG_MODULE_REGISTER(read_Vsupply);

#define VBATT DT_PATH(vbatt)				
#define ZEPHYR_USER DT_PATH(zephyr_user)  	

#ifdef CONFIG_BOARD_THINGY52_NRF52832
	/* This board uses a divider that reduces max voltage to
 	* reference voltage (600 mV).
 	*/
	#define BATTERY_ADC_GAIN ADC_GAIN_1
#else
	/* Other boards may use dividers that only reduce battery voltage to
 	* the maximum supported by the hardware (3.6 V)
 	*/
	#define BATTERY_ADC_GAIN ADC_GAIN_1_6	
#endif
 
struct io_channel_config {
	uint8_t channel;
};

struct divider_config {
	struct io_channel_config io_channel;
	struct gpio_dt_spec power_gpios;
	/* output_ohm is used as a flag value: if it is nonzero then
	 * the battery is measured through a voltage divider;
	 * otherwise it is assumed to be directly connected to Vdd.
	*/
	uint32_t output_ohm;
	uint32_t full_ohm;
};


static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
	.io_channel = {
		DT_IO_CHANNELS_INPUT(VBATT),
	},
	.power_gpios = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {}), 
	.output_ohm = DT_PROP(VBATT, output_ohms),
	.full_ohm = DT_PROP(VBATT, full_ohms),
#else /* /vbatt not exists */	
	.io_channel = {
		DT_IO_CHANNELS_INPUT(ZEPHYR_USER),
	},
#endif /* /vbatt exists */
};


struct divider_data {
	const struct device *adc;
	struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	int16_t raw;
};


static struct divider_data divider_data = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
	.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(VBATT)),
#else
	.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER)),
#endif
};

/**
 * @brief Configure ADC and GPIO for voltage divider measurement
 *
 * Internal setup function that:
 * 1. Verifies ADC device is ready
 * 2. Configures power GPIO if present (for enabling measurement circuit)
 * 3. Sets up ADC sequence with oversampling and calibration
 * 4. Configures channel for appropriate input (divider or VDD)
 *
 * @return 0 on success, negative error code on failure
 */
static int16_t divider_setup(void)
{
	const struct divider_config *cfg = &divider_config;
	const struct io_channel_config *iocp = &cfg->io_channel;
	const struct gpio_dt_spec *gcp = &cfg->power_gpios;
	struct divider_data *ddp = &divider_data;
	struct adc_sequence *asp = &ddp->adc_seq;
	struct adc_channel_cfg *accp = &ddp->adc_cfg;
	int16_t rc;

	if (!device_is_ready(ddp->adc)) {
		LOG_ERR("ADC device is not ready %s", ddp->adc->name);
		return -ENOENT;
	}

	if (gcp->port) {
		if (!device_is_ready(gcp->port)) {
			LOG_ERR("%s: device not ready", gcp->port->name);
			return -ENOENT;
		}
		rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
		if (rc != 0) {
			LOG_ERR("Failed to control feed %s.%u: %d",
				gcp->port->name, gcp->pin, rc);
			return rc;
		}
	}


	*asp = (struct adc_sequence){
		.channels = BIT(0),
		.buffer = &ddp->raw,
		.buffer_size = sizeof(ddp->raw),
		.oversampling = 4,
		.calibrate = true,
	};

#ifdef CONFIG_ADC_NRFX_SAADC
	*accp = (struct adc_channel_cfg){
		.gain = BATTERY_ADC_GAIN,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	};

	if (cfg->output_ohm != 0) {
		accp->input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
			+ iocp->channel;			
	} else {
		accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
	}

	asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

	rc = adc_channel_setup(ddp->adc, accp);
	if (ENABLE_PRINT)
		LOG_INF("Setup AIN%u got %d", iocp->channel, rc);

	return rc;
}

static bool battery_ok;

/**
 * @brief Initialize battery/supply voltage measurement
 *
 * Calls divider_setup() and tracks whether initialization was successful.
 * Must be called before battery_sample() can return valid readings.
 *
 * @return 0 on success, negative error code on failure
 */
int16_t battery_setup(void)
{
	int16_t rc = divider_setup();

	battery_ok = (rc == 0);
	if (ENABLE_PRINT)
		LOG_INF("Battery setup: %d %d", rc, battery_ok);
	return rc;
}

/**
 * @brief Enable or disable the voltage measurement circuit
 *
 * Controls the power GPIO that enables the voltage divider circuit
 * for measurement. Should be enabled before sampling and disabled
 * after to save power.
 *
 * @param enable true to enable measurement, false to disable
 * @return 0 on success, -ENOENT if not initialized, or GPIO error code
 */
int16_t battery_measure_enable(bool enable)
{
	int16_t rc = -ENOENT;

	if (battery_ok) {
		const struct gpio_dt_spec *gcp = &divider_config.power_gpios;

		rc = 0;
		if (gcp->port) {
			rc = gpio_pin_set_dt(gcp, enable);
		}
	}
	return rc;
}

/**
 * @brief Take a single voltage sample
 *
 * Performs one ADC reading and converts to millivolts. If a voltage
 * divider is configured, applies the divider ratio to calculate
 * actual voltage.
 *
 * @note battery_setup() must be called first and battery_measure_enable(true)
 *       should be called before sampling for accurate readings.
 *
 * @return Voltage in millivolts, or negative error code on failure
 */
int16_t battery_sample(void)
{
	int16_t rc = -ENOENT;

	if (battery_ok) {
		struct divider_data *ddp = &divider_data;
		const struct divider_config *dcp = &divider_config;
		struct adc_sequence *sp = &ddp->adc_seq;

		rc = adc_read(ddp->adc, sp);
		sp->calibrate = false;
		if (rc == 0) {
			int32_t val = ddp->raw;

			adc_raw_to_millivolts(adc_ref_internal(ddp->adc),
					      ddp->adc_cfg.gain,
					      sp->resolution,
					      &val);

			if (dcp->output_ohm != 0) {
				rc = val * (uint64_t)dcp->full_ohm
					/ dcp->output_ohm;
				if (ENABLE_PRINT)
					LOG_INF("raw %u ~ %u mV => %d mV",
					ddp->raw, val, rc);
			} else {
				rc = val;
				// if (ENABLE_PRINT)
				// 	LOG_INF("raw %u ~ %u mV", ddp->raw, val);
			}
		}
	}

	return rc;
}

/**
 * @brief Read MCU supply voltage
 *
 * Performs a complete measurement cycle of the supply voltage that
 * powers the MCU. Used when operating without a boost converter.
 *
 * The function:
 * 1. Initializes the measurement circuit (battery_setup)
 * 2. Enables the measurement circuit
 * 3. Takes 5 consecutive samples with 20ms delays
 * 4. Averages the readings for noise reduction
 * 5. Disables the measurement circuit to save power
 *
 * @return Supply voltage in millivolts, or 0 on initialization failure
 */
uint16_t read_Vsupp_mv(void) {
	battery_setup();
	uint16_t rc = battery_measure_enable(true);
  	int32_t batt_mV = 0;
	k_sleep(K_MSEC(10));
	if (rc != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", rc);
		return 0;
	}


  for (uint8_t i = 0; i < 5; i++)
  {
    batt_mV += battery_sample();
    k_sleep(K_MSEC(20));
  }
  batt_mV = batt_mV/5;
  
	if (batt_mV < 0) {
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
	} else {
		if (ENABLE_PRINT)
			LOG_INF("- Battery voltage: %d mV", batt_mV);
	}

	battery_measure_enable(false);
	return (uint16_t) batt_mV;
}