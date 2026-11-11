/**
 * @file main.c
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
 * @brief Firmware composition root, dependency injection, and non-blocking superloop.
 */

#include "command_dispatcher.h"
#include "frame_parser.h"
#include "gait_controller.h"
#include "icomm.h"
#include "iimu.h"
#include "iservo.h"
#include "mem_pool.h"
#include "motion_planner.h"
#include "mpu6050.h"
#include "pca9685.h"
#include "timer.h"
#include "uart.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>

enum {
	POOL_BLOCK_SIZE = 16, /**< 16 bytes per block (accommodates motion_keyframe_t). */
	POOL_BLOCK_COUNT = 16 /**< 16 blocks = 256 bytes static SRAM pool. */
};

/* Static memory allocation for MISRA-C compliance */
static uint8_t s_pool_storage[POOL_BLOCK_COUNT * POOL_BLOCK_SIZE];
static uint8_t s_pool_free_stack[POOL_BLOCK_COUNT];

static mem_pool_t s_mem_pool;
static motion_planner_t s_motion_planner;
static gait_controller_t s_gait_controller;
static frame_parser_t s_frame_parser;
static command_dispatcher_t s_command_dispatcher;

int main(void) {
	cli();
	wdt_enable(WDTO_2S);

	/* 1. Initialize static Memory Pool */
	(void
	)mem_pool_init(&s_mem_pool, s_pool_storage, s_pool_free_stack, (uint8_t)POOL_BLOCK_SIZE, (uint8_t)POOL_BLOCK_COUNT);

	/* 2. Initialize Hardware HAL Drivers */
	(void)uart_init((uint32_t)UART_DEFAULT_BAUD_RATE);
	(void)timer_init();
	(void)pca9685_init((const pca9685_config_t *)0);
	(void)mpu6050_init((const mpu6050_config_t *)0);

	/* 3. Obtain Abstract Ports */
	comm_stream_t stream = uart_get_stream();
	servo_driver_t servo = pca9685_get_driver();
	imu_driver_t imu = mpu6050_get_driver();

	/* 4. Initialize Domain & Application Controllers (Dependency Injection) */
	(void)motion_planner_init(&s_motion_planner, &s_mem_pool, &servo);
	(void)gait_controller_init(&s_gait_controller);
	frame_parser_init(&s_frame_parser);
	(void)command_dispatcher_init(&s_command_dispatcher, stream, &s_motion_planner, &s_gait_controller, &s_mem_pool);
	command_dispatcher_set_imu(&s_command_dispatcher, &imu);

	sei();

	/* 5. Deterministic Non-Blocking Superloop */
	while (1) {
		wdt_reset();

		/* Step A: Drain UART RX stream and feed binary protocol state machine */
		while (comm_available(&stream)) {
			uint8_t const byte = comm_read(&stream);
			protocol_frame_t frame;
			if (frame_parser_feed_byte(&s_frame_parser, byte, &frame)) {
				command_dispatcher_dispatch(&s_command_dispatcher, &frame);
			}
		}

		/* Step B: Periodic 20ms (50 Hz) motion control cycle */
		if (timer_check_20ms_tick()) {
			gait_controller_step(&s_gait_controller, &s_motion_planner);
			(void)motion_planner_step(&s_motion_planner);
		}
	}

	return 0;
}
