//=============================================================================#
/**
 * @brief: Solar Panel Voltage Divider Switch Control
 *
 * Controls a digital switch that enables a voltage divider for
 * reading the solar panel open-circuit voltage
 */
//=============================================================================#

#ifndef APPLICATION_SWITCH_VPV_DIVIDER_H
#define APPLICATION_SWITCH_VPV_DIVIDER_H

#include <stdbool.h>
#include <stdint.h>

int8_t check_gpio_Vpv_divider(void);
void enable_Vpv_divider(void);
void disable_Vpv_divider(void);

// Query current Vpv divider state
bool is_Vpv_divider_enabled(void);

#endif  /*APPLICATION_SWITCH_VPV_DIVIDER_H*/

