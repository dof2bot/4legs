/**
 * @file motion_planner.c
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
 * @brief Joint angle interpolation sequencer and trajectory queue.
 */

#include "motion_planner.h"

static inline void step_joint(int16_t *current, int16_t target, int16_t step) {
	int16_t const diff = target - *current;
	if (diff == 0) {
		return;
	}
	if (step == 0) {
		*current += (diff > 0) ? 1 : -1;
	} else if (((step > 0) && (diff > step)) ||
	           ((step < 0) && (diff < step))) {
		*current += step;
	} else {
		*current = target;
	}
}

bool motion_planner_init(
	motion_planner_t *planner, mem_pool_t *pool, const servo_driver_t *servo
) {
	if ((planner == (motion_planner_t *)0) || (pool == (mem_pool_t *)0) ||
	    (servo == (const servo_driver_t *)0)) {
		return false;
	}

	planner->pool = pool;
	planner->servo = servo;
	planner->speed_multiplier = (uint8_t)MOTION_SPEED_DEFAULT;
	planner->queue_head = 0U;
	planner->queue_tail = 0U;
	planner->queue_count = 0U;

	/* Default boot (sitting) stance joint angles in tenths of a degree */
	static const int16_t
		default_boot_angles[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG] = {
	    {1228, 306, 443},  /* Leg 0: FR */
	    {572, 1494, 1357}, /* Leg 1: FL */
	    {900, 1528, 1459}, /* Leg 2: BR */
	    {900, 272, 341}    /* Leg 3: BL */
	};

	for (uint8_t i = 0U; i < (uint8_t)ROBOT_LEGS_COUNT; ++i) {
		for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; ++j) {
			planner->angle_now[i][j] = default_boot_angles[i][j];
			planner->angle_target[i][j] = default_boot_angles[i][j];
			planner->step_speed[i][j] = 0;
			int16_t const a = default_boot_angles[i][j];
			(void)servo_set_angle_tenths(planner->servo, i, j, a);
		}
	}

	return true;
}

bool motion_planner_set_joint_target(
    motion_planner_t *planner,
    uint8_t leg,
    int16_t coxa_tenths,
    int16_t femur_tenths,
    int16_t tibia_tenths,
    uint16_t duration_ms
) {
	if ((planner == (motion_planner_t *)0) ||
	    (leg >= (uint8_t)ROBOT_LEGS_COUNT)) {
		return false;
	}

	planner->angle_target[leg][JOINT_COXA] = coxa_tenths;
	planner->angle_target[leg][JOINT_FEMUR] = femur_tenths;
	planner->angle_target[leg][JOINT_TIBIA] = tibia_tenths;

	uint16_t const interval = (uint16_t)MOTION_STEP_INTERVAL_MS;
	uint16_t steps = (uint16_t)(duration_ms / interval);
	if (steps == 0U) {
		steps = 1U;
	}

	/* Scale steps by speed multiplier (1..10) */
	uint32_t const num =
		(uint32_t)steps * (uint32_t)MOTION_SPEED_DEFAULT;
	steps = (uint16_t)(num / (uint32_t)planner->speed_multiplier);
	if (steps == 0U) {
		steps = 1U;
	}

	int16_t const targets[ROBOT_JOINTS_PER_LEG] = {
		coxa_tenths, femur_tenths, tibia_tenths
	};
	for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; ++j) {
		int16_t const diff = targets[j] - planner->angle_now[leg][j];
		planner->step_speed[leg][j] = (int16_t)(diff / (int16_t)steps);
	}

	return true;
}

bool motion_planner_is_reached(const motion_planner_t *planner) {
	if (planner == (const motion_planner_t *)0) {
		return true;
	}

	for (uint8_t i = 0U; i < (uint8_t)ROBOT_LEGS_COUNT; ++i) {
		for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; ++j) {
			if (planner->angle_now[i][j] !=
			    planner->angle_target[i][j]) {
				return false;
			}
		}
	}

	return true;
}

bool motion_planner_step(motion_planner_t *planner) {
	if (planner == (motion_planner_t *)0) {
		return false;
	}

	bool moving = false;

	/* Step interpolation towards targets */
	for (uint8_t i = 0U; i < (uint8_t)ROBOT_LEGS_COUNT; ++i) {
		for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; ++j) {
			int16_t *const cur = &planner->angle_now[i][j];
			int16_t const tgt = planner->angle_target[i][j];
			if (*cur != tgt) {
				moving = true;
				step_joint(cur, tgt, planner->step_speed[i][j]);
				(void)servo_set_angle_tenths(
					planner->servo, i, j, *cur
				);
			}
		}
	}

	/* If current targets are reached, advance trajectory queue */
	if ((!moving) && (planner->queue_count > 0U)) {
		uint8_t const tail = planner->queue_tail;
		const motion_keyframe_t *const kf = planner->queue[tail];

		if (kf != (const motion_keyframe_t *)0) {
			int16_t const cx = kf->target_coxa;
			int16_t const fm = kf->target_femur;
			int16_t const tb = kf->target_tibia;
			uint16_t const dur = kf->duration_ms;
			uint8_t const legs = (uint8_t)ROBOT_LEGS_COUNT;
			if (kf->leg_index < legs) {
				(void)motion_planner_set_joint_target(
					planner, kf->leg_index, cx, fm, tb, dur
				);
			} else if (kf->leg_index == (uint8_t)LEG_ALL) {
				for (uint8_t l = 0U; l < legs; ++l) {
					(void)motion_planner_set_joint_target(
						planner, l, cx, fm, tb, dur
					);
				}
			} else {
				/* Invalid leg index */
			}

			/* Release keyframe block back to memory pool */
			(void)mem_pool_free(planner->pool, kf);
			planner->queue[tail] = (motion_keyframe_t *)0;
		}

		uint8_t const cap = (uint8_t)MOTION_QUEUE_CAPACITY;
		planner->queue_tail = (uint8_t)((tail + 1U) % cap);
		planner->queue_count--;
		moving = true;
	}

	return moving;
}

bool motion_planner_enqueue_keyframe(
	motion_planner_t *planner, const motion_keyframe_t *keyframe
) {
	if ((planner == (motion_planner_t *)0) ||
	    (keyframe == (const motion_keyframe_t *)0)) {
		return false;
	}

	uint8_t const cap = (uint8_t)MOTION_QUEUE_CAPACITY;
	if (planner->queue_count >= cap) {
		return false; /* Queue full */
	}

	/* Allocate fixed-size block from memory pool */
	motion_keyframe_t *block =
		(motion_keyframe_t *)mem_pool_alloc(planner->pool);
	if (block == (motion_keyframe_t *)0) {
		return false; /* Memory pool exhausted */
	}

	/* Copy keyframe payload into pool-allocated block */
	*block = *keyframe;

	planner->queue[planner->queue_head] = block;
	planner->queue_head = (uint8_t)((planner->queue_head + 1U) % cap);
	planner->queue_count++;

	return true;
}

bool motion_planner_clear_queue(motion_planner_t *planner) {
	if (planner == (motion_planner_t *)0) {
		return false;
	}

	uint8_t const cap = (uint8_t)MOTION_QUEUE_CAPACITY;
	while (planner->queue_count > 0U) {
		uint8_t const tail = planner->queue_tail;
		const motion_keyframe_t *const kf = planner->queue[tail];
		if (kf != (const motion_keyframe_t *)0) {
			(void)mem_pool_free(planner->pool, kf);
			planner->queue[tail] = (motion_keyframe_t *)0;
		}
		planner->queue_tail = (uint8_t)((tail + 1U) % cap);
		planner->queue_count--;
	}

	planner->queue_head = 0U;
	planner->queue_tail = 0U;
	return true;
}

uint8_t motion_planner_queue_count(const motion_planner_t *planner) {
	if (planner == (const motion_planner_t *)0) {
		return 0U;
	}
	return planner->queue_count;
}

void motion_planner_set_speed(motion_planner_t *planner, uint8_t speed) {
	if (planner != (motion_planner_t *)0) {
		if (speed < (uint8_t)MOTION_SPEED_MIN) {
			speed = (uint8_t)MOTION_SPEED_MIN;
		} else if (speed > (uint8_t)MOTION_SPEED_MAX) {
			speed = (uint8_t)MOTION_SPEED_MAX;
		} else {
			/* in range */
		}
		planner->speed_multiplier = speed;
	}
}
