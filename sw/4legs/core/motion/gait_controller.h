/**
 * @file gait_controller.h
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
 * @brief Non-blocking Finite State Machine (FSM) gait and choreography controller.
 */

#pragma once

#include "motion_planner.h"
#include "motion_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	robot_state_t state;
	gait_type_t active_gait;
	uint8_t step_phase;
	uint8_t steps_remaining;
	bool is_standing;
} gait_controller_t;

/**
 * @brief Initializes the gait controller.
 *
 * @param controller Pointer to gait_controller_t.
 * @return bool true on success.
 */
bool gait_controller_init(gait_controller_t *controller);

/**
 * @brief Requests execution of a predefined gait pattern.
 *
 * @param controller Pointer to gait_controller_t.
 * @param gait Gait type to execute.
 * @param count Number of steps or repetitions.
 * @return bool true if request accepted, false if busy with another gait.
 */
bool gait_controller_request(gait_controller_t *controller, gait_type_t gait, uint8_t count);

/**
 * @brief Advances the gait FSM when current leg interpolation targets are reached.
 *
 * @param controller Pointer to gait_controller_t.
 * @param planner Pointer to active motion planner.
 */
void gait_controller_step(gait_controller_t *controller, motion_planner_t *planner);

/**
 * @brief Checks if gait controller is currently idle.
 *
 * @param controller Pointer to gait_controller_t.
 * @return bool true if idle.
 */
bool gait_controller_is_idle(const gait_controller_t *controller);

/**
 * @brief Immediately aborts current gait and halts in place.
 *
 * @param controller Pointer to gait_controller_t.
 * @param planner Pointer to active motion planner.
 */
void gait_controller_abort(gait_controller_t *controller, motion_planner_t *planner);

/**
 * @brief Returns current robot posture state.
 *
 * @param controller Pointer to gait_controller_t.
 * @return robot_state_t Current state.
 */
robot_state_t gait_controller_get_state(const gait_controller_t *controller);

/**
 * @brief Returns current active gait type.
 *
 * @param controller Pointer to gait_controller_t.
 * @return gait_type_t Active gait.
 */
gait_type_t gait_controller_get_active_gait(const gait_controller_t *controller);

#ifdef __cplusplus
}
#endif
