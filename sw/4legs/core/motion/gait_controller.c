/**
 * @file gait_controller.c
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
 * @brief Finite State Machine (FSM) gait controller implementation.
 */

#include "gait_controller.h"

enum {
	GAIT_DEFAULT_TRANSITION_MS = 200U,
	GAIT_BODY_TRANSITION_MS = 300U,
	GAIT_POSE_TRANSITION_MS = 400U
};

/* Calibrated default standing pose joint angles in tenths of a degree */
static const int16_t POSE_STAND[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG] = {
    {1228, 600, 590},  /* Leg 0: FR */
    {572, 1200, 1210}, /* Leg 1: FL */
    {900, 1185, 1288}, /* Leg 2: BR */
    {900, 615, 512}    /* Leg 3: BL */
};

/* Calibrated default sitting / boot pose joint angles in tenths of a degree */
static const int16_t POSE_SIT[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG] = {
    {1228, 306, 443},  /* Leg 0: FR */
    {572, 1494, 1357}, /* Leg 1: FL */
    {900, 1528, 1459}, /* Leg 2: BR */
    {900, 272, 341}    /* Leg 3: BL */
};

static inline void set_target(
	motion_planner_t *p, uint8_t leg,
	int16_t c, int16_t f, int16_t t, uint16_t dur
) {
	(void)motion_planner_set_joint_target(p, leg, c, f, t, dur);
}

bool gait_controller_init(gait_controller_t *controller) {
	if (controller == (gait_controller_t *)0) {
		return false;
	}

	controller->state = ROBOT_STATE_SITTING;
	controller->active_gait = GAIT_SIT;
	controller->step_phase = 0U;
	controller->steps_remaining = 0U;
	controller->is_standing = false;

	return true;
}

bool gait_controller_request(
	gait_controller_t *controller, gait_type_t gait, uint8_t count
) {
	if (controller == (gait_controller_t *)0) {
		return false;
	}

	if (controller->steps_remaining > 0U) {
		return false; /* Busy */
	}

	controller->active_gait = gait;
	controller->steps_remaining = (count > 0U) ? count : 1U;
	controller->step_phase = 0U;

	if (gait == GAIT_SIT) {
		controller->state = ROBOT_STATE_SITTING;
		controller->is_standing = false;
	} else if (gait == GAIT_STAND) {
		controller->state = ROBOT_STATE_STANDING;
		controller->is_standing = true;
	} else if ((gait == GAIT_HAND_WAVE) || (gait == GAIT_HAND_SHAKE)) {
		controller->state = ROBOT_STATE_ACTION;
	} else {
		controller->state = ROBOT_STATE_WALKING;
	}

	return true;
}

bool gait_controller_is_idle(const gait_controller_t *controller) {
	if (controller == (const gait_controller_t *)0) {
		return true;
	}

	return (controller->steps_remaining == 0U);
}

void gait_controller_abort(
	gait_controller_t *controller, motion_planner_t *planner
) {
	if (controller != (gait_controller_t *)0) {
		controller->steps_remaining = 0U;
		controller->step_phase = 0U;
		controller->state = ROBOT_STATE_EMERGENCY_STOP;
	}
	if (planner != (motion_planner_t *)0) {
		(void)motion_planner_clear_queue(planner);
	}
}

robot_state_t gait_controller_get_state(const gait_controller_t *controller) {
	if (controller == (const gait_controller_t *)0) {
		return ROBOT_STATE_EMERGENCY_STOP;
	}
	return controller->state;
}

gait_type_t gait_controller_get_active_gait(
	const gait_controller_t *controller
) {
	if (controller == (const gait_controller_t *)0) {
		return GAIT_SIT;
	}
	return controller->active_gait;
}

static void step_forward_fsm(
	gait_controller_t *controller, motion_planner_t *planner
) {
	uint16_t const d_ms = (uint16_t)GAIT_DEFAULT_TRANSITION_MS;
	uint16_t const b_ms = (uint16_t)GAIT_BODY_TRANSITION_MS;
	uint8_t const br = (uint8_t)LEG_BACK_RIGHT;
	uint8_t const fl = (uint8_t)LEG_FRONT_LEFT;
	uint8_t const fr = (uint8_t)LEG_FRONT_RIGHT;
	uint8_t const bl = (uint8_t)LEG_BACK_LEFT;

	switch (controller->step_phase) {
	case 0U:
		/* Leg 2 lift */
		set_target(planner, br, 900, 1492, 1445, d_ms);
		controller->step_phase++;
		break;
	case 1U:
		/* Leg 2 swing forward */
		set_target(planner, br, 378, 1353, 1084, d_ms);
		controller->step_phase++;
		break;
	case 2U:
		/* Leg 2 plant down */
		set_target(planner, br, 378, 1155, 974, d_ms);
		controller->step_phase++;
		break;
	case 3U:
		/* Shift body forward across all legs */
		set_target(planner, fr, 900, 615, 512, b_ms);
		set_target(planner, fl, 378, 1155, 974, b_ms);
		set_target(planner, br, 572, 1200, 1210, b_ms);
		set_target(planner, bl, 1228, 600, 590, b_ms);
		controller->step_phase++;
		break;
	case 4U:
		/* Leg 1 lift */
		set_target(planner, fl, 378, 1353, 1084, d_ms);
		controller->step_phase++;
		break;
	case 5U:
		/* Leg 1 swing backward to neutral */
		set_target(planner, fl, 900, 1492, 1445, d_ms);
		controller->step_phase++;
		break;
	case 6U:
		/* Leg 1 plant down */
		set_target(planner, fl, 900, 1185, 1288, d_ms);
		controller->step_phase++;
		break;
	case 7U:
	default:
		controller->step_phase = 0U;
		if (controller->steps_remaining > 0U) {
			controller->steps_remaining--;
		}
		if (controller->steps_remaining == 0U) {
			controller->state = ROBOT_STATE_STANDING;
		}
		break;
	}
}

void gait_controller_step(
	gait_controller_t *controller, motion_planner_t *planner
) {
	if ((controller == (gait_controller_t *)0) ||
	    (planner == (motion_planner_t *)0)) {
		return;
	}

	/* Advance FSM only when interpolation reached target */
	if (!motion_planner_is_reached(planner)) {
		return;
	}

	if (controller->steps_remaining == 0U) {
		return;
	}

	switch (controller->active_gait) {
	case GAIT_STAND:
		for (uint8_t i = 0U; i < (uint8_t)ROBOT_LEGS_COUNT; ++i) {
			(void)motion_planner_set_joint_target(
			    planner,
			    i,
			    POSE_STAND[i][JOINT_COXA],
			    POSE_STAND[i][JOINT_FEMUR],
			    POSE_STAND[i][JOINT_TIBIA],
			    (uint16_t)GAIT_POSE_TRANSITION_MS
			);
		}
		controller->steps_remaining = 0U;
		controller->is_standing = true;
		controller->state = ROBOT_STATE_STANDING;
		break;

	case GAIT_SIT:
		for (uint8_t i = 0U; i < (uint8_t)ROBOT_LEGS_COUNT; ++i) {
			(void)motion_planner_set_joint_target(
			    planner,
			    i,
			    POSE_SIT[i][JOINT_COXA],
			    POSE_SIT[i][JOINT_FEMUR],
			    POSE_SIT[i][JOINT_TIBIA],
			    (uint16_t)GAIT_POSE_TRANSITION_MS
			);
		}
		controller->steps_remaining = 0U;
		controller->is_standing = false;
		controller->state = ROBOT_STATE_SITTING;
		break;

	case GAIT_STEP_FORWARD:
		step_forward_fsm(controller, planner);
		break;

	case GAIT_HAND_WAVE:
		{
			uint16_t const b_ms = (uint16_t)GAIT_BODY_TRANSITION_MS;
			uint8_t const fr = (uint8_t)LEG_FRONT_RIGHT;
			if (controller->step_phase == 0U) {
				set_target(planner, fr, 1074, 0, 340, b_ms);
				controller->step_phase = 1U;
			} else if (controller->step_phase == 1U) {
				set_target(planner, fr, 1492, 0, 334, b_ms);
				controller->step_phase = 2U;
			} else {
				set_target(
					planner, fr,
					POSE_STAND[fr][JOINT_COXA],
					POSE_STAND[fr][JOINT_FEMUR],
					POSE_STAND[fr][JOINT_TIBIA],
					b_ms
				);
		}
			controller->step_phase = 0U;
			if (controller->steps_remaining > 0U) {
				controller->steps_remaining--;
			}
			if (controller->steps_remaining == 0U) {
				controller->state = ROBOT_STATE_STANDING;
			}
		}
		break;

	default:
		controller->steps_remaining = 0U;
		controller->state = ROBOT_STATE_STANDING;
		break;
	}
}
