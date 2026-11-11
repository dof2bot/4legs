/**
 * @file motion_planner.h
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
 * @brief Non-blocking linear joint angle interpolation sequencer and trajectory queue.
 *
 * Coordinates 50Hz (20ms) discrete joint angle interpolation across all 4 legs,
 * manages the FIFO keyframe queue backed by the static memory pool, and drives
 * the abstract iservo_driver_t port with zero floating-point operations.
 */

#pragma once

#include "iservo.h"
#include "mem_pool.h"
#include "motion_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	MOTION_QUEUE_CAPACITY = 16,   /**< Maximum number of keyframes queued in trajectory. */
	MOTION_STEP_INTERVAL_MS = 20, /**< Motion planner discrete step interval (20ms = 50Hz). */
	MOTION_SPEED_MIN = 1,         /**< Minimum speed multiplier index (0.2x). */
	MOTION_SPEED_MAX = 10,        /**< Maximum speed multiplier index (2.0x). */
	MOTION_SPEED_DEFAULT = 5      /**< Default speed multiplier index (1.0x). */
};

/**
 * @brief Motion planner control context structure.
 */
typedef struct {
	int16_t angle_now[ROBOT_LEGS_COUNT]
	                 [ROBOT_JOINTS_PER_LEG]; /**< Current joint angles in tenths of a deg (0..1800). */
	int16_t angle_target[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG]; /**< Target joint angles in tenths of a deg. */
	int16_t step_speed[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG];   /**< Delta angle per 20ms step in tenths of a deg. */
	uint8_t speed_multiplier; /**< Global speed multiplier index (1..10, 5 = 1.0x). */

	mem_pool_t *pool;                                /**< Backing memory pool for keyframes. */
	motion_keyframe_t *queue[MOTION_QUEUE_CAPACITY]; /**< FIFO queue of keyframe pointers. */
	uint8_t queue_head;                              /**< Queue push index. */
	uint8_t queue_tail;                              /**< Queue pop index. */
	uint8_t queue_count;                             /**< Number of active queued keyframes. */

	const servo_driver_t *servo; /**< Actuator output port. */
} motion_planner_t;

/**
 * @brief Initializes the motion planner to default boot pose.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @param pool Pointer to initialized memory pool.
 * @param servo Pointer to servo driver port.
 * @return bool true on success.
 */
bool motion_planner_init(motion_planner_t *planner, mem_pool_t *pool, const servo_driver_t *servo);

/**
 * @brief Sets target joint angles for a specific leg and computes step speed.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @param leg Leg index (0..3).
 * @param coxa_tenths Target Coxa angle in tenths of a degree (0..1800).
 * @param femur_tenths Target Femur angle in tenths of a degree (0..1800).
 * @param tibia_tenths Target Tibia angle in tenths of a degree (0..1800).
 * @param duration_ms Desired transition duration in milliseconds (0 for immediate).
 * @return bool true on success.
 */
bool motion_planner_set_joint_target(
    motion_planner_t *planner,
    uint8_t leg,
    int16_t coxa_tenths,
    int16_t femur_tenths,
    int16_t tibia_tenths,
    uint16_t duration_ms
);

/**
 * @brief Checks if all 4 legs have reached their target joint angles.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @return bool true if all targets reached.
 */
bool motion_planner_is_reached(const motion_planner_t *planner);

/**
 * @brief Executes one discrete 20ms motion interpolation step across all legs.
 *
 * Interpolates joint angles using integer arithmetic and updates servo outputs.
 * Dispatches the next keyframe from the queue when current target is reached.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @return bool true if motion is actively in progress, false if idle.
 */
bool motion_planner_step(motion_planner_t *planner);

/**
 * @brief Enqueues a keyframe into the trajectory queue from the memory pool.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @param keyframe Pointer to motion_keyframe_t data to copy.
 * @return bool true if enqueued, false if queue or pool is full.
 */
bool motion_planner_enqueue_keyframe(motion_planner_t *planner, const motion_keyframe_t *keyframe);

/**
 * @brief Clears all queued keyframes and returns blocks to the memory pool.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @return bool true on success.
 */
bool motion_planner_clear_queue(motion_planner_t *planner);

/**
 * @brief Retrieves the number of keyframes currently queued.
 *
 * @param planner Pointer to motion_planner_t instance.
 * @return uint8_t Queued keyframe count.
 */
uint8_t motion_planner_queue_count(const motion_planner_t *planner);

/**
 * @brief Sets global speed multiplier (1..10).
 *
 * @param planner Pointer to motion_planner_t instance.
 * @param speed Speed level (1=slowest, 10=fastest, 5=1.0x).
 */
void motion_planner_set_speed(motion_planner_t *planner, uint8_t speed);

#ifdef __cplusplus
}
#endif
