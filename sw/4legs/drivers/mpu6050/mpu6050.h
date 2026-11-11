/**
 * @file mpu6050.h
 * Copyright (C) 2026 Vladimir Roncevic <elektron.ronca@gmail.com>
 *
 * 4legs is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * 4legs is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program_name. If not, see <http://www.gnu.org/licenses/>.
 *
 * @brief Hardware driver for InvenSense MPU-6050 6-DOF IMU sensor over TWI/I2C.
 */

#pragma once

#include "iimu.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	MPU6050_I2C_ADDR_DEFAULT = 0x68U, /**< Default AD0 low address. */
	MPU6050_I2C_ADDR_ALT = 0x69U      /**< Alternative AD0 high address. */
};

typedef struct {
	uint8_t i2c_addr; /**< 7-bit I2C slave address. */
} mpu6050_config_t;

/**
 * @brief Initializes MPU-6050 hardware sensor, clears sleep mode and configures filters.
 *
 * @param config Optional configuration pointer (NULL for default 0x68).
 * @return bool true if sensor responded and initialized, false otherwise.
 */
bool mpu6050_init(const mpu6050_config_t *config);

/**
 * @brief Reads acceleration, attitude angles, and gyro rate from MPU-6050.
 *
 * @param context Driver instance context.
 * @param data Destination pointer for telemetry data.
 * @return bool true on success, false on communication failure.
 */
bool mpu6050_read_data(void *context, imu_data_t *data);

/**
 * @brief Checks if MPU-6050 driver is initialized and operational.
 *
 * @param context Driver instance context.
 * @return bool true if ready, false otherwise.
 */
bool mpu6050_is_ready(void *context);

/**
 * @brief Obtains polymorphic IMU driver port.
 *
 * @return imu_driver_t Structure containing ops pointer and context.
 */
imu_driver_t mpu6050_get_driver(void);

#ifdef __cplusplus
}
#endif

