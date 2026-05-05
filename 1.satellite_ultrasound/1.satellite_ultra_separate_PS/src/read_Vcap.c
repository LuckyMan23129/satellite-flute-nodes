/**
 * @file read_Vcap.c
 * @brief Supercapacitor Voltage Measurement
 *
 * This module provides ADC-based voltage measurement of the storage
 * supercapacitor (Vcap). The voltage is measured directly via an analog
 * input pin connected to the capacitor.
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
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "config.h"
#include "read_Vcap.h"

LOG_MODULE_REGISTER(read_Vcap);

// ADC configuration (implementation details)
#define ADC_RESOLUTION         14
#define ADC_GAIN               ADC_GAIN_1_6
#define ADC_REFERENCE          ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME   ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID         1
#define ADC_CHANNEL_INPUT      SAADC_CH_PSELP_PSELP_AnalogInput1  // AIN1

#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

/**
 * @brief Initialize ADC for supercapacitor voltage measurement
 *
 * Verifies that the ADC device is ready for use. Must be called
 * before using read_Vcap_mv().
 *
 * @return 0 on success, -1 if ADC device is not ready
 */
int8_t Vcap_init(void) {
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -1;
    }
    return 0;
}

/**
 * @brief Read supercapacitor voltage
 *
 * Performs 5 consecutive ADC readings and returns the average voltage
 * in millivolts. Uses 12-bit resolution with internal reference.
 *
 * The function:
 * 1. Configures the ADC channel with appropriate gain and reference
 * 2. Takes 5 samples with brief delays between readings
 * 3. Filters out invalid readings (negative or >30000 raw)
 * 4. Converts raw values to millivolts
 * 5. Returns the average of valid readings
 *
 * @return Capacitor voltage in millivolts, or 30000 on error
 */
uint16_t read_Vcap_mv(void)
{   
    Vcap_init();
    
    int16_t sample_buffer;  // stack-local: each concurrent call has its own copy
    struct adc_channel_cfg channel_cfg = {
        .gain             = ADC_GAIN,
        .reference        = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id       = ADC_CHANNEL_ID,
        
        .input_positive   = ADC_CHANNEL_INPUT,
        // .differential     = 0,
    };

    if (adc_channel_setup(adc_dev, &channel_cfg)) {
        LOG_ERR("ADC channel setup failed");
        return 30000;
    }

    const struct adc_sequence sequence = {
        .channels    = BIT(ADC_CHANNEL_ID),
        .buffer      = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = ADC_RESOLUTION,
    };

    k_sleep(K_MSEC(10));

    int32_t avg_mv = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        sample_buffer = 0;
        int8_t err = adc_read(adc_dev, &sequence);
        k_sleep(K_MSEC(20));
        if (err != 0) {
            LOG_ERR("ADC reading failed with error %d", err);
            return 30000;
        }

        int16_t raw = sample_buffer;
        if ((raw < 0) || (raw>30000)) raw = 0;
        int32_t mv = raw;
        adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &mv);
        // Should HIGHER THAN 15ms - This delay is very important
		k_sleep(K_MSEC(20));
        avg_mv += mv;
    }
    avg_mv = avg_mv/5;
    return (uint16_t)avg_mv;
}