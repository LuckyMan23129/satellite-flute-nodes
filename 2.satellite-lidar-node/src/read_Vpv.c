/**
 * @file read_Vpv.c
 * @brief Solar Panel Voltage Measurement
 *
 * This module provides ADC-based voltage measurement of the solar panel
 * open-circuit voltage (Vpv). The measurement requires temporarily isolating
 * the solar panel from the storage capacitor and enabling a voltage divider.
 *
 * 
 * Measurement Process:
 * 1. Disable charging (isolate solar panel from capacitor)
 * 2. Enable voltage divider
 * 3. Wait for voltage to stabilize (200ms)
 * 4. Read ADC and multiply by 2 (divider ratio)
 * 5. Disable voltage divider
 * 6. Re-enable charging
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

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "read_Vpv.h"
#include "opencircuit.h"
#include "switch_Vpv_divider.h"

LOG_MODULE_REGISTER(read_solar);

// ADC configuration (implementation details)
#define ADC_NUM_CHANNELS        1
#define ADC_NODE                DT_PHANDLE(DT_PATH(zephyr_user), io_channels)
#define ADC_RESOLUTION          12
#define ADC_GAIN                ADC_GAIN_1_6
#define ADC_REFERENCE           ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME    ADC_ACQ_TIME_DEFAULT

int32_t sample_buffer[ADC_NUM_CHANNELS];
int32_t avg_reading[ADC_NUM_CHANNELS];


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
 	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
 	#error "No suitable devicetree overlay specified"
 #endif


 /* Get the numbers of up to two channels */
static uint8_t channel_ids[ADC_NUM_CHANNELS] = {
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 0),
#if ADC_NUM_CHANNELS == 2
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 1)
#endif
};

// V- Used to configure each specific channel's parameters
struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = 0,
	.differential = 0
};

/**
 * @brief: V- This struct is ONLY used to specify a "buffer" where an ADC readings is saved
 */
	struct adc_sequence sequence = {
	.channels    = 0,
	.buffer      = sample_buffer,
	.buffer_size = sizeof(sample_buffer),		/* buffer size in bytes, not number of samples */
	.resolution  = ADC_RESOLUTION,
};


/**
 * @brief Read raw ADC value from solar panel voltage divider
 *
 * Performs ADC reading of the solar panel voltage through the voltage
 * divider circuit. Takes 5 consecutive readings and returns the average.
 *
 * The function:
 * 1. Verifies ADC device is ready
 * 2. Configures the ADC channel for the appropriate input
 * 3. Takes 5 samples with delays for stability
 * 4. Filters out invalid readings
 * 5. Converts to millivolts and averages
 *
 * @note This returns the divided voltage. Caller must multiply by
 *       divider ratio (2x) to get actual solar panel voltage.
 *
 * @return ADC reading in millivolts (after divider), 1 on error
 */
uint16_t read_adc(void)
{
	int err;
	const struct device *dev_adc = DEVICE_DT_GET(ADC_NODE);

	if (!device_is_ready(dev_adc)) {
			LOG_ERR("ADC device not found");
		return 1;
	}
	sequence.channels = 0;
	for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
		channel_cfg.channel_id = channel_ids[i];
		#ifdef CONFIG_ADC_NRFX_SAADC
			channel_cfg.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + channel_ids[i];    // Specify the input pin for ADC channel
		#endif
		adc_channel_setup(dev_adc, &channel_cfg);
		k_sleep(K_MSEC(5));
		sequence.channels |= BIT(channel_ids[i]);
	}

	int32_t adc_vref = adc_ref_internal(dev_adc);
	k_sleep(K_MSEC(5));

	if (ENABLE_PRINT)
    	LOG_INF("Start reading V_Solar_Pannels...");
	memset(avg_reading, 0, sizeof(avg_reading));
    for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
		avg_reading[i] = 0;
        for (uint8_t m = 0; m < 5; m++)
		{
			memset(sample_buffer, 0, sizeof(sample_buffer));
			err = adc_read(dev_adc, &sequence);
			k_sleep(K_MSEC(20));
			if (err != 0) {
				LOG_ERR("ADC reading failed with error %d", err);
				return 1;
			}

			// Convert raw reading to millivolts
			int32_t raw_value = sample_buffer[i];
			if (((raw_value < 0) || (raw_value > 30000)))
				raw_value = 0;
        	if (adc_vref > 0) {
				int32_t mv_value = raw_value;
				adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);
				
				// Should HIGHER THAN 15ms - This delay is very important
				k_sleep(K_MSEC(20));
				
				avg_reading[i] += mv_value;
			}
        }
    }

	// Get average  of consecutive 5-time readings
	avg_reading[0] = avg_reading[0]/5;
	return (uint16_t)avg_reading[0];
}

/**
 * @brief Measure solar panel open-circuit voltage
 *
 * Performs a complete measurement cycle of the solar panel voltage.
 * This function manages all the hardware switching needed for accurate
 * open-circuit voltage measurement.
 *
 * Measurement sequence:
 * 1. Disable charging (isolate solar panel from capacitor via SW2)
 * 2. Enable voltage divider (SW3)
 * 3. Wait 200ms for voltage stabilization
 * 4. Read ADC and multiply by 2 (voltage divider ratio)
 * 5. Disable voltage divider (SW3)
 * 6. Re-enable charging (SW2)
 *
 * @note The 200ms delay is critical for accurate measurement as it allows
 *       the solar panel voltage to stabilize after isolation.
 *
 * @return Solar panel open-circuit voltage in millivolts
 */
uint16_t update_Vpv(void)
{
    disable_charging();       					// Isolate the solar panels from Capacitors to measure open-circuit Vpv
    enable_Vpv_divider();
	
	// Should HIGHER THAN 200ms - This delay is very important
    k_sleep(K_MSEC(200));						// This delay is very important
	
	uint16_t Vpv = 2*(read_adc());
	disable_Vpv_divider();
    /* Charging reconnection is intentionally left to the caller (schedule()).
     * Reconnecting here -- before the caller reads a fresh Vcap -- can cause a
     * large inrush current spike when Vsolar >> Vcap (e.g. Vsolar > 6000 mV),
     * which disturbs the power rail and can drop the LTE-M modem connection. */
    if (ENABLE_PRINT)
      LOG_INF("Voltage of Solar Panels - Vpv = %d mV", Vpv);

	return (Vpv);
}
