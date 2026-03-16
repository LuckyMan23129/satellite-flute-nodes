//============================================================================================
/**
  * @brief: Control a digital switch to power on/off peripherals (e.g. sensors)
  *         to save energy when not in use
*/
//============================================================================================


#ifndef APPLICATION_SWITCH_PERIPHERAL_POWER_H
#define APPLICATION_SWITCH_PERIPHERAL_POWER_H

#include <stdint.h>

/**
 * @brief Initialize GPIO for peripheral power switch
 * @return 0 on success, 1 on failure (GPIO not ready or configuration failed)
 */
int8_t check_gpio_periph_sw(void);

/**
 * @brief Turn on the digital switch to power the peripherals
 *
 * Powers on peripherals (e.g. sensors) so that data can be read from them.
 */
void periph_sw_turn_on(void);

/**
 * @brief Turn off the digital switch to power down the peripherals
 *
 * Powers off peripherals (e.g. sensors) to save energy when not in use.
 */
void periph_sw_turn_off(void);

#endif /*APPLICATION_SWITCH_PERIPHERAL_POWER_H*/