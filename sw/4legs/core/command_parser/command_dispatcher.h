/**
 * @file command_dispatcher.h
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
 * @brief Command execution dispatcher and telemetry response serializer.
 */

#pragma once

#include "frame_parser.h"
#include "gait_controller.h"
#include "icomm.h"
#include "iimu.h"
#include "mem_pool.h"
#include "motion_planner.h"
#include "protocol_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	comm_stream_t stream;
	motion_planner_t *planner;
	gait_controller_t *gait;
	mem_pool_t *pool;
	const imu_driver_t *imu;
	protocol_err_code_t last_error;
} command_dispatcher_t;

/**
 * @brief Initializes the command dispatcher.
 *
 * @param disp Pointer to command_dispatcher_t.
 * @param stream Abstract communication stream.
 * @param planner Motion planner instance.
 * @param gait Gait controller instance.
 * @param pool Memory pool instance.
 * @return bool true on success.
 */
bool command_dispatcher_init(
    command_dispatcher_t *disp,
    comm_stream_t stream,
    motion_planner_t *planner,
    gait_controller_t *gait,
    mem_pool_t *pool
);

/**
 * @brief Registers optional IMU sensor driver for telemetry queries.
 *
 * @param disp Pointer to command_dispatcher_t.
 * @param imu Pointer to imu_driver_t port.
 */
void command_dispatcher_set_imu(command_dispatcher_t *disp, const imu_driver_t *imu);

/**
 * @brief Dispatches and executes an incoming validated binary protocol frame.
 *
 * @param disp Pointer to command_dispatcher_t.
 * @param frame Pointer to validated protocol_frame_t.
 */
void command_dispatcher_dispatch(command_dispatcher_t *disp, const protocol_frame_t *frame);

/**
 * @brief Sends a response frame over communication stream.
 *
 * @param disp Pointer to command_dispatcher_t.
 * @param resp_id Response message ID.
 * @param payload Payload bytes.
 * @param length Payload length.
 */
void command_dispatcher_send_response(
    command_dispatcher_t *disp, protocol_resp_id_t resp_id, const uint8_t *payload, uint8_t length
);

#ifdef __cplusplus
}
#endif
