#ifndef APPLICATION_I2C_SENSOR4_H_
#define APPLICATION_I2C_SENSOR4_H_

#include <zephyr/drivers/i2c.h>

// ---------------------------------------------------------------------------
// I2C address and register map
// ---------------------------------------------------------------------------
#define LIDARLITE_ADDR_DEFAULT  0x62   /* default I2C address of LiDAR Lite */
#define LIDAR4_CONFIG_REG       0x03   /* sensor configuration register      */
#define LIDAR4_HIGH_REG         0x0f   /* distance high byte                 */
#define LIDAR4_LOW_REG          0x10   /* distance low byte                  */

// ---------------------------------------------------------------------------
// Buffer size constants  (must stay in sync with the UDP server)
// ---------------------------------------------------------------------------
#define RAW_READING_NUMBERS       100  /* total samples per measurement pass */
#define RAW_READING_NUMBERS_SENT   10  /* samples included in UDP frame      */

// ---------------------------------------------------------------------------
// Public API — low-level primitives
// ---------------------------------------------------------------------------
int8_t  check_i2c_access(void);
int8_t  get_nodeID_i2c(void);
int8_t  pre_conf(int8_t mode);       /* select sensor operating mode        */
int16_t take_range(void);            /* trigger a distance measurement      */
uint8_t getBusyFlag(void);
void    waitForBusy(void);
int8_t  config_lidar(void);          /* I2C check + pre_conf(7)             */
int16_t readDistance(void);          /* read two-byte distance result       */
int16_t distance_data_Func(void);    /* waitForBusy→take_range→readDistance */

// ---------------------------------------------------------------------------
// Public API — high-level acquisition
// ---------------------------------------------------------------------------

/**
 * @brief Acquire, average, and validate a water-level reading from the LiDAR.
 *
 * @param buff_sent  Output array of RAW_READING_NUMBERS_SENT uint16_t values.
 *                
 * @return           Calibrated mean water level in cm.
 */
uint16_t lidar_read_water_level(void);

#endif /* APPLICATION_I2C_SENSOR4_H_ */