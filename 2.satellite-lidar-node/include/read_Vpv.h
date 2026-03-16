//=============================================================================#
/**
 * @brief: Read Voltage of solar Panels
 *
 * This module provides functions to read the solar panel open-circuit voltage
 * using the nRF ADC (SAADC) on AIN0.
 */
//=============================================================================#

#ifndef APPICATION_READ_VSOLAR_H_
#define APPICATION_READ_VSOLAR_H_

#include <stdint.h>

/**
 * @brief Read raw ADC value from solar panel voltage divider
 * @return Voltage in millivolts (average of 5 samples)
 */
uint16_t read_adc(void);

/**
 * @brief Update and return solar panel open-circuit voltage
 *
 * This function temporarily isolates the solar panel from capacitors,
 * enables the voltage divider, reads the voltage, then restores charging.
 *
 * @return Solar panel voltage in millivolts (Vpv)
 */
uint16_t update_Vpv(void);

#endif  /*APPICATION_READ_VSOLAR_H*/
