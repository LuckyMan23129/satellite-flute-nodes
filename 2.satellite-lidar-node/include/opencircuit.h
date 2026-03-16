//=============================================================================#
/**
 * @brief: Solar Panel Charging Control
 *
 * Controls the digital switch that connects or disconnects the solar panel
 * from the storage capacitor. Stops charging when Vcap exceeds Vmax to
 * protect the system from overvoltage.
 */
//=============================================================================#


#ifndef APPICATION_OPEN_CIRCUIT_H
#define APPICATION_OPEN_CIRCUIT_H

#include <stdbool.h>
#include <stdint.h>

int8_t check_gpio_sw_opencircuit(void);

// Turn on the Digital Switch to connect solar panels to charge the Caps
void enable_charging(void);

// Turn off the Digital Switch to isolate solar panels from the Caps
void disable_charging(void);

// Query current charging state
bool is_charging_enabled(void);

#endif  /*APPICATION_OPEN_CIRCUIT_H*/

