/**
 * @file iservo.h
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
 * @brief Abstract multi-channel servo driver interface (Output Port / DIP).
 *
 * Decouples kinematically computed joint angles from physical PWM pulse generation
 * across the 12 servo channels (4 legs x 3 joints: Coxa, Femur, Tibia).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Quadruped robot leg index enumeration.
 */
typedef enum {
	LEG_FRONT_RIGHT = 0,  /**< Leg 0: Front-Right leg. */
	LEG_FRONT_LEFT = 1,   /**< Leg 1: Front-Left leg. */
	LEG_BACK_RIGHT = 2,   /**< Leg 2: Back-Right leg. */
	LEG_BACK_LEFT = 3,    /**< Leg 3: Back-Left leg. */
	ROBOT_LEGS_COUNT = 4, /**< Total number of legs on quadruped robot. */
	LEG_ALL = 0xFF        /**< Broadcast sentinel index addressing all legs. */
} robot_leg_t;

/**
 * @brief Degrees of freedom (joints) per leg enumeration.
 */
typedef enum {
	JOINT_COXA = 0,          /**< Coxa joint index (horizontal hip sweep). */
	JOINT_FEMUR = 1,         /**< Femur joint index (thigh pitch). */
	JOINT_TIBIA = 2,         /**< Tibia joint index (knee pitch). */
	ROBOT_JOINTS_PER_LEG = 3 /**< Number of joints per leg. */
} robot_joint_t;

enum {
	TOTAL_SERVOS_COUNT = 12, /**< Total number of servo actuators (4 * 3). */

	SERVO_MIN_PULSE_US = 500,  /**< Minimum allowable PWM pulse width in microseconds. */
	SERVO_MID_PULSE_US = 1500, /**< Neutral (90 degree) center PWM pulse width. */
	SERVO_MAX_PULSE_US = 2500, /**< Maximum allowable PWM pulse width in microseconds. */

	SERVO_MIN_ANGLE_DEG = 0,  /**< Minimum servo shaft angle (degrees). */
	SERVO_MID_ANGLE_DEG = 90, /**< Neutral 90 degree servo shaft angle. */
	SERVO_MAX_ANGLE_DEG = 180 /**< Maximum 180 degree servo shaft angle. */
};

typedef struct servo_driver servo_driver_t;

/**
 * @brief Table of virtual method pointers for servo actuation.
 */
typedef struct {
	bool (*set_pulse_us)(void *context, uint8_t leg, uint8_t joint, uint16_t pulse_us);
	bool (*set_angle_tenths)(void *context, uint8_t leg, uint8_t joint, int16_t tenths);
	bool (*set_angle_deg)(void *context, uint8_t leg, uint8_t joint, float angle_deg);
	bool (*attach)(void *context);
	bool (*detach)(void *context);
	bool (*is_attached)(void *context);
} servo_ops_t;

/**
 * @brief Servo driver polymorphic encapsulation.
 */
struct servo_driver {
	const servo_ops_t *ops; /**< Virtual method table pointer. */
	void *context;          /**< Concrete hardware driver context (e.g. pca9685_t). */
};

static inline bool servo_set_pulse_us(const servo_driver_t *driver, uint8_t leg, uint8_t joint, uint16_t pulse_us) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->set_pulse_us != (bool (*)(void *, uint8_t, uint8_t, uint16_t))0)) {
		return driver->ops->set_pulse_us(driver->context, leg, joint, pulse_us);
	}
	return false;
}

static inline bool servo_set_angle_tenths(const servo_driver_t *driver, uint8_t leg, uint8_t joint, int16_t tenths) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->set_angle_tenths != (bool (*)(void *, uint8_t, uint8_t, int16_t))0)) {
		return driver->ops->set_angle_tenths(driver->context, leg, joint, tenths);
	}
	return false;
}

static inline bool servo_set_angle_deg(const servo_driver_t *driver, uint8_t leg, uint8_t joint, float angle_deg) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->set_angle_deg != (bool (*)(void *, uint8_t, uint8_t, float))0)) {
		return driver->ops->set_angle_deg(driver->context, leg, joint, angle_deg);
	}
	return false;
}

static inline bool servo_attach(const servo_driver_t *driver) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->attach != (bool (*)(void *))0)) {
		return driver->ops->attach(driver->context);
	}
	return false;
}

static inline bool servo_detach(const servo_driver_t *driver) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->detach != (bool (*)(void *))0)) {
		return driver->ops->detach(driver->context);
	}
	return false;
}

static inline bool servo_is_attached(const servo_driver_t *driver) {
	if ((driver != (const servo_driver_t *)0) && (driver->ops != (const servo_ops_t *)0) &&
	    (driver->ops->is_attached != (bool (*)(void *))0)) {
		return driver->ops->is_attached(driver->context);
	}
	return false;
}

#ifdef __cplusplus
}
#endif
