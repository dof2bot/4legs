/**
 * @file iimu.h
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
 * @brief Abstract 6-DOF IMU sensor driver interface (Input Port / DIP).
 *
 * Decouples attitude estimation and motion telemetry from specific
 * hardware IMU sensors (e.g. MPU6050, LSM6DS3).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Consolidated 6-DOF IMU telemetry attitude and acceleration data.
 */
typedef struct {
	int16_t roll;    /**< Roll attitude angle in tenths of a degree (-1800..1800). */
	int16_t pitch;   /**< Pitch attitude angle in tenths of a degree (-900..900). */
	int16_t accel_x; /**< Linear acceleration along X axis in milli-g (mg). */
	int16_t accel_y; /**< Linear acceleration along Y axis in milli-g (mg). */
	int16_t accel_z; /**< Linear acceleration along Z axis in milli-g (mg). */
	int16_t gyro_z;  /**< Angular velocity around Z axis (yaw rate) in tenths of deg/s. */
} imu_data_t;

typedef struct imu_driver imu_driver_t;

/**
 * @brief Virtual method table for IMU sensor operations.
 */
typedef struct {
	bool (*read_data)(void *context, imu_data_t *data);
	bool (*is_ready)(void *context);
} imu_ops_t;

/**
 * @brief IMU driver polymorphic encapsulation.
 */
struct imu_driver {
	const imu_ops_t *ops; /**< Virtual method table pointer. */
	void *context;        /**< Concrete hardware driver context. */
};

static inline bool imu_read_data(const imu_driver_t *driver, imu_data_t *data) {
	if ((driver != (const imu_driver_t *)0) && (driver->ops != (const imu_ops_t *)0) &&
	    (driver->ops->read_data != (bool (*)(void *, imu_data_t *))0)) {
		return driver->ops->read_data(driver->context, data);
	}
	return false;
}

static inline bool imu_is_ready(const imu_driver_t *driver) {
	if ((driver != (const imu_driver_t *)0) && (driver->ops != (const imu_ops_t *)0) &&
	    (driver->ops->is_ready != (bool (*)(void *))0)) {
		return driver->ops->is_ready(driver->context);
	}
	return false;
}

#ifdef __cplusplus
}
#endif

