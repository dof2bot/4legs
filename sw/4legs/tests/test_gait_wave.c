/* Host test: the hand wave runs all three phases and ends in the stand pose.
 * Build and run from sw/4legs:
 *   cc -std=c99 -Icore/motion -Icore/interfaces -Icore/mem_pool \
 *      tests/test_gait_wave.c core/motion/gait_controller.c \
 *      core/motion/motion_planner.c core/mem_pool/mem_pool.c -o /tmp/t && /tmp/t
 */
#include "gait_controller.h"

#include <stdio.h>

static bool set_tenths(void *c, uint8_t l, uint8_t j, int16_t t) {
	(void)c; (void)l; (void)j; (void)t;
	return true;
}

static const servo_ops_t ops = {0, set_tenths, 0, 0, 0, 0};

int main(void) {
	static uint8_t storage[MOTION_QUEUE_CAPACITY * sizeof(motion_keyframe_t)];
	static uint8_t free_stack[MOTION_QUEUE_CAPACITY];
	servo_driver_t servo = {&ops, 0};
	mem_pool_t pool;
	motion_planner_t mp;
	gait_controller_t gc;
	int peak_coxa = 0;

	(void)mem_pool_init(&pool, storage, free_stack,
			    (uint8_t)sizeof(motion_keyframe_t), MOTION_QUEUE_CAPACITY);
	(void)motion_planner_init(&mp, &pool, &servo);
	(void)gait_controller_init(&gc);
	(void)gait_controller_request(&gc, GAIT_HAND_WAVE, 1U);

	for (int tick = 0; tick < 2000; ++tick) {
		gait_controller_step(&gc, &mp);
		(void)motion_planner_step(&mp);
		if (mp.angle_now[LEG_FRONT_RIGHT][JOINT_COXA] > peak_coxa) {
			peak_coxa = mp.angle_now[LEG_FRONT_RIGHT][JOINT_COXA];
		}
	}

	/* phase 1 swings the coxa to 149.2 deg; afterwards the leg is back down */
	int const ok = (peak_coxa == 1492) &&
		       (mp.angle_now[LEG_FRONT_RIGHT][JOINT_FEMUR] == 600) &&
		       (mp.angle_now[LEG_FRONT_RIGHT][JOINT_TIBIA] == 590);
	printf("peak coxa %d, final FR femur %d tibia %d: %s\n", peak_coxa,
	       mp.angle_now[LEG_FRONT_RIGHT][JOINT_FEMUR],
	       mp.angle_now[LEG_FRONT_RIGHT][JOINT_TIBIA], ok ? "PASS" : "FAIL");
	return ok ? 0 : 1;
}
