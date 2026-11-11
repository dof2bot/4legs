/**
 * @file motion_types.h
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
 * @brief Kinematic and motion planning data types.
 */

#pragma once

#include "iservo.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	SCALE_FACTOR_TENTHS = 10 /**< Scale factor for tenths of a millimeter (e.g. 625 = 62.5 mm). */
};

/**
 * @brief 3D Cartesian coordinate in millimeters.
 */
typedef struct {
	float x;
	float y;
	float z;
} point3d_t;

/**
 * @brief 3D Cartesian coordinate using fixed-point tenths of a millimeter (int16_t).
 *
 * Example: 62.5 mm -> 625.
 */
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} point3d_i16_t;

/**
 * @brief 3-DOF Joint angles in degrees.
 */
typedef struct {
	float alpha; /**< Coxa angle in degrees (0..180). */
	float beta;  /**< Femur angle in degrees (0..180). */
	float gamma; /**< Tibia angle in degrees (0..180). */
} joint_angles_t;

/**
 * @brief 3-DOF Joint angles in tenths of a degree (0..1800).
 */
typedef struct {
	int16_t coxa;  /**< Coxa angle in tenths of a degree (0..1800). */
	int16_t femur; /**< Femur angle in tenths of a degree (0..1800). */
	int16_t tibia; /**< Tibia angle in tenths of a degree (0..1800). */
} joint_angles_i16_t;

/**
 * @brief High-level robot operational state.
 */
typedef enum {
	ROBOT_STATE_SITTING = 0,
	ROBOT_STATE_STANDING = 1,
	ROBOT_STATE_WALKING = 2,
	ROBOT_STATE_ACTION = 3,
	ROBOT_STATE_EMERGENCY_STOP = 4
} robot_state_t;

/**
 * @brief Predefined gait identifiers.
 */
typedef enum {
	GAIT_SIT = 0,
	GAIT_STAND = 1,
	GAIT_STEP_FORWARD = 2,
	GAIT_STEP_BACKWARD = 3,
	GAIT_TURN_LEFT = 4,
	GAIT_TURN_RIGHT = 5,
	GAIT_HAND_WAVE = 6,
	GAIT_HAND_SHAKE = 7
} gait_type_t;

/**
 * @brief Motion keyframe element allocated from memory pool.
 *
 * Encapsulates 3-DOF joint angle targets in tenths of a degree (0..1800)
 * and interpolation duration. Fits within a compact 16-byte pool block!
 */
typedef struct {
	uint8_t leg_index;    /**< Target leg index (0..3), or 0xFF for all legs. */
	int16_t target_coxa;  /**< Target Coxa angle in tenths of a degree (0..1800). */
	int16_t target_femur; /**< Target Femur angle in tenths of a degree (0..1800). */
	int16_t target_tibia; /**< Target Tibia angle in tenths of a degree (0..1800). */
	uint16_t duration_ms; /**< Duration to reach target in milliseconds. */
	uint8_t flags;        /**< Execution flags / options. */
} motion_keyframe_t;

#ifdef __cplusplus
}
#endif
