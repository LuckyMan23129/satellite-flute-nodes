//============================================================================================
/** 
  * @brief: Enable/Disable CHARGING SUPER-CAPACITORS when Vcap > Vmax to protect the system      
*/
//============================================================================================


#ifndef APPLICATION_SWITCH_OPEN_CIRCUIT_H
#define APPLICATION_SWITCH_OPEN_CIRCUIT_H

#include <stdbool.h>
#include <stdint.h>

int8_t check_gpio_sw_opencircuit(void);

// Turn on the Digital Switch to connect solar panels to charge the Caps
void enable_charging(void);

// Turn off the Digital Switch to isolate solar panels from the Caps
void disable_charging(void);

// Query current charging state
bool is_charging_enabled(void);

#endif  /*APPLICATION_SWITCH_OPEN_CIRCUIT_H*/