#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "i2c_sensor4.h"
#include "config.h"


LOG_MODULE_REGISTER(i2c_sensor4);  

static int8_t  ret;
static uint16_t distance;

uint8_t sigCountMax     = 0;
uint8_t acqConfigReg    = 0;
uint8_t refCountMax     = 0;
uint8_t thresholdBypass = 0;
uint8_t ACQ_COMMAND     = 0;


// ===========================================================================
// I2C Initialisation
// ===========================================================================

#define I2C2_NODE DT_NODELABEL(my_lidar4)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);

int8_t check_i2c_access(void)
{
    if (ENABLE_PRINT)
        LOG_INF("LIDAR_Lite_v3HP - Checking I2C DT access...");
    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("LIDAR_Lite_v3HP - I2C bus %s is not ready!", dev_i2c.bus->name);
        return 1;
    }
    return 0;
}


// ===========================================================================
// Sensor Configuration
// ===========================================================================

int8_t pre_conf(int8_t mode)
{
    if (ENABLE_PRINT)
        LOG_INF("LIDAR_Lite_v3HP - Choose the Sensor Mode ...");
    switch (mode)
    {
        case 0: // Default mode, balanced performance
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            thresholdBypass = 0x00;
            break;

        case 1: // Short range, high speed
            sigCountMax     = 0x1d;
            acqConfigReg    = 0x08;
            thresholdBypass = 0x00;
            break;

        case 2: // Default range, higher speed short range
            sigCountMax     = 0x80;
            acqConfigReg    = 0x00;
            thresholdBypass = 0x00;
            break;

        case 3: // Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08;
            thresholdBypass = 0x00;
            break;

        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            thresholdBypass = 0x80;
            break;

        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            thresholdBypass = 0xb0;
            break;

        case 6: // Short range, high speed, higher error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x01;
            thresholdBypass = 0x00;
            break;

        case 7: // Custom — low sensitivity, low erroneous measurements
            sigCountMax     = 0xff;  // REG0x02: max sample count
            acqConfigReg    = 0x0C;  // REG0x04: enables REF count register
            refCountMax     = 0xff;  // REG0x12: max reference acquisitions
            thresholdBypass = 0x99;  // EXCELLENT: error +-4 cm
            ACQ_COMMAND     = 0x04;  // REG0x00: acquisition with bias correction
            break;
    }

    // Write sigCountMax → register 0x02
    char buff1[] = {0x02, sigCountMax};
    ret = i2c_write_dt(&dev_i2c, buff1, sizeof(buff1));
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write addr %x reg %x", dev_i2c.addr, buff1[0]);
        return 1;
    }

    // Write acqConfigReg → register 0x04
    char buff2[] = {0x04, acqConfigReg};
    ret = i2c_write_dt(&dev_i2c, buff2, sizeof(buff2));
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write addr %x reg %x", dev_i2c.addr, buff2[0]);
        return 1;
    }

    // Write refCountMax → register 0x12
    char buff3[] = {0x12, refCountMax};
    ret = i2c_write_dt(&dev_i2c, buff3, sizeof(buff3));
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write addr %x reg %x", dev_i2c.addr, buff3[0]);
        return 1;
    }

    // Write thresholdBypass → register 0x1c
    char buff4[] = {0x1c, thresholdBypass};
    ret = i2c_write_dt(&dev_i2c, buff4, sizeof(buff4));
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write addr %x reg %x", dev_i2c.addr, buff4[0]);
        return 1;
    }

    return 0;
}


// ===========================================================================
// Low-level Measurement Primitives
// ===========================================================================

int16_t take_range(void)
{
    char buff5[] = {0x00, ACQ_COMMAND};
    ret = i2c_write_dt(&dev_i2c, buff5, sizeof(buff5));
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write addr %x reg %x", dev_i2c.addr, buff5[0]);
        return 1;
    }
    return 0;
}

uint8_t getBusyFlag(void)
{
    uint8_t busy_flag           = 1;
    uint8_t busyFlag[1]         = {0};
    uint8_t busyFlag_regs[1]    = {0x01};

    ret = i2c_write_read_dt(&dev_i2c, &busyFlag_regs[0], 1, &busyFlag[0], 1);
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write/read addr %x reg %x",
               dev_i2c.addr, busyFlag_regs[0]);
    }
    busy_flag = busyFlag[0] & 0x01;
    return busy_flag;
}

void waitForBusy(void)
{
    uint16_t busyCounter = 0;
    uint8_t  check_flag  = 1;
    while (check_flag) {
        if (busyCounter > 9999) break;
        check_flag = getBusyFlag();
        busyCounter++;
    }
    if (busyCounter > 9999)
        LOG_ERR(">>>> timeout of waitForBusy()");
}

int16_t readDistance(void)
{
    uint8_t distance_reading[2] = {0};
    uint8_t sensor_regs[2]      = {LIDAR4_LOW_REG, LIDAR4_HIGH_REG};

    ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[0], 1, &distance_reading[0], 1);
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write/read addr %x reg %x",
               dev_i2c.addr, sensor_regs[0]);
        return -1;
    }
    ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[1], 1, &distance_reading[1], 1);
    if (ret != 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to write/read addr %x reg %x",
               dev_i2c.addr, sensor_regs[1]);
        return -1;
    }

    distance = (distance_reading[1] << 8) | distance_reading[0];
    return distance;
}

int8_t config_lidar(void)
{
    ret = check_i2c_access();
    if (ret) return -1;
    ret = pre_conf(7);
    if (ret) return -1;
    return 0;
}

int16_t distance_data_Func(void)
{
    int16_t distance_data = 0;
    waitForBusy();
    take_range();
    waitForBusy();
    distance_data = readDistance();
    if (distance_data < 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to read distance data");
        return -1;
    }
    return distance_data;
}


// ===========================================================================
// lidar_read_water_level()
// ===========================================================================

/**
 * @brief Acquire RAW_READING_NUMBERS LiDAR distance readings and return
 *        the calibrated average.
 *
 * @param buff_sent  Output buffer; must be at least RAW_READING_NUMBERS_SENT
 *                   uint16_t entries. Filled with raw readings for transmission.
 * @return           Calibrated mean water level in cm.
 */
uint16_t lidar_read_water_level(void)
{
    /* Persists across calls for outlier detection */
    static uint16_t old_distance = 0;

    /* Local raw sample buffer — no longer a global */
    uint16_t lidar_buff[RAW_READING_NUMBERS];

    int32_t  sum      = 0;
    uint16_t distance_cm = 0;

    int8_t ret = config_lidar();
    if (ret < 0) {
        LOG_ERR("LIDAR_Lite_v3HP - Failed to configure LiDAR");
        return 59999;
    }
    
    memset(lidar_buff, 0, sizeof(lidar_buff));
    for (uint16_t i = 0; i < RAW_READING_NUMBERS; i++) {
        lidar_buff[i] = distance_data_Func();
        k_sleep(K_MSEC(5));
    }

    for (uint16_t n = 0; n < RAW_READING_NUMBERS; n++) {
        sum += lidar_buff[n];
    }
    distance_cm = (sum / RAW_READING_NUMBERS) - 10;  /* -10 cm calibration */

    // ── Outlier rejection: re-measure if jump > 15 cm ────────────────────
    if (abs((int32_t)old_distance - (int32_t)distance_cm) >= 15)
    {
        config_lidar();
        memset(lidar_buff, 0, sizeof(lidar_buff));
        for (uint16_t i = 0; i < RAW_READING_NUMBERS; i++) {
            lidar_buff[i] = distance_data_Func();
            k_sleep(K_MSEC(2));
        }

        sum = 0;
        for (uint16_t n = 0; n < RAW_READING_NUMBERS; n++) {
            sum += lidar_buff[n];
        }
        distance_cm = (sum / RAW_READING_NUMBERS) - 10;
    }

    old_distance = distance_cm;
    if (ENABLE_PRINT)
        LOG_INF("LIDAR_Lite_v3HP - Mean Water Level: %d cm", distance_cm);

    return distance_cm;
}