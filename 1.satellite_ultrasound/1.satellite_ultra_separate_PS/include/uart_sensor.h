/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//=============================================================================#
/**
 * @file uart_sensor.h
 * @brief Ultrasonic Distance Sensor (UART1) Public API
 *
 * Public interface for the ultrasonic distance sensor driver. The sensor
 * communicates over UART1 at 9600 baud and reports distance as a 3-byte
 * frame: two data bytes (big-endian mm) followed by one checksum byte.
 */
//=============================================================================#

#ifndef APPLICATION_UART_SENSOR_H_
#define APPLICATION_UART_SENSOR_H_

/* uart_cb is static (internal to uart_sensor.c) — not part of the public API */
int8_t initialize_uart_sensor(void);
bool check_data_available(void);
uint16_t read_distance(void);
uint16_t distance_cm();
uint16_t get_safe_distance_cm(void);

#endif /* APPLICATION_UART_SENSOR_H_ */

