//=============================================================================#
/**
 * @brief: Read Voltage of the super-capacitor
 *
 * This module provides functions to initialize and read the supercapacitor
 * voltage using the nRF ADC (SAADC) on AIN1.
 *
 * @note: Vcap != Vsupply when using a boost converter
 */
//=============================================================================#

#ifndef APPICATION_READ_VCAP_H_
#define APPICATION_READ_VCAP_H_

#include <stdint.h>

/**
 * @brief Initialize the Vcap ADC
 * @return 0 on success, -1 on failure
 */
int8_t Vcap_init(void);

/**
 * @brief Read the supercapacitor voltage
 * @return Voltage in millivolts (average of 5 samples)
 */
uint16_t read_Vcap_mv(void);

#endif /*APPICATION_READ_VCAP_H_*/
