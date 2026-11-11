/**
 * @file command_dispatcher.c
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
 * @brief Command execution dispatcher and telemetry response serializer implementation.
 */

#include "command_dispatcher.h"

#include "crc8.h"

enum {
	DEFAULT_DIRECT_MOVE_MS = 200U
};

bool command_dispatcher_init(
    command_dispatcher_t *disp,
    comm_stream_t stream,
    motion_planner_t *planner,
    gait_controller_t *gait,
    mem_pool_t *pool
) {
	if ((disp == (command_dispatcher_t *)0) || (planner == (motion_planner_t *)0) || (gait == (gait_controller_t *)0) ||
	    (pool == (mem_pool_t *)0)) {
		return false;
	}

	disp->stream = stream;
	disp->planner = planner;
	disp->gait = gait;
	disp->pool = pool;
	disp->imu = (const imu_driver_t *)0;
	disp->last_error = ERR_NONE;

	return true;
}

void command_dispatcher_set_imu(command_dispatcher_t *disp, const imu_driver_t *imu) {
	if (disp != (command_dispatcher_t *)0) {
		disp->imu = imu;
	}
}

void command_dispatcher_send_response(
    command_dispatcher_t *disp, protocol_resp_id_t resp_id, const uint8_t *payload, uint8_t length
) {
	if (disp == (command_dispatcher_t *)0) {
		return;
	}

	uint8_t crc = 0U;
	(void)comm_write(&disp->stream, (uint8_t)PROTOCOL_SYNC_BYTE);

	(void)comm_write(&disp->stream, (uint8_t)resp_id);
	crc = crc8_update(crc, (uint8_t)resp_id);

	(void)comm_write(&disp->stream, length);
	crc = crc8_update(crc, length);

	for (uint8_t i = 0U; i < length; ++i) {
		(void)comm_write(&disp->stream, payload[i]);
		crc = crc8_update(crc, payload[i]);
	}

	(void)comm_write(&disp->stream, crc);
}

static void send_ack(command_dispatcher_t *disp, uint8_t cmd_id) {
	uint8_t payload = cmd_id;
	command_dispatcher_send_response(disp, RESP_ACK, &payload, (uint8_t)PAYLOAD_LEN_ACK);
}

static void send_nack(command_dispatcher_t *disp, uint8_t cmd_id, protocol_err_code_t err) {
	disp->last_error = err;
	uint8_t payload[PAYLOAD_LEN_NACK];
	payload[0] = cmd_id;
	payload[1] = (uint8_t)err;
	command_dispatcher_send_response(disp, RESP_NACK, payload, (uint8_t)PAYLOAD_LEN_NACK);
}

void command_dispatcher_dispatch(command_dispatcher_t *disp, const protocol_frame_t *frame) {
	if ((disp == (command_dispatcher_t *)0) || (frame == (const protocol_frame_t *)0)) {
		return;
	}

	switch ((protocol_cmd_id_t)frame->msg_id) {
	case CMD_PING:
		command_dispatcher_send_response(disp, RESP_PONG, (const uint8_t *)0, (uint8_t)PAYLOAD_LEN_PONG);
		break;

	case CMD_STOP:
		gait_controller_abort(disp->gait, disp->planner);
		send_ack(disp, (uint8_t)CMD_STOP);
		break;

	case CMD_SET_SPEED:
		if (frame->length >= (uint8_t)PAYLOAD_LEN_SET_SPEED) {
			motion_planner_set_speed(disp->planner, frame->payload[0]);
			send_ack(disp, (uint8_t)CMD_SET_SPEED);
		} else {
			send_nack(disp, frame->msg_id, ERR_BAD_LENGTH);
		}
		break;

	case CMD_GAIT_STEP:
		if (frame->length >= (uint8_t)PAYLOAD_LEN_GAIT_STEP) {
			gait_type_t const gait = (gait_type_t)frame->payload[0];
			uint8_t const count = frame->payload[1];
			if (gait_controller_request(disp->gait, gait, count)) {
				send_ack(disp, (uint8_t)CMD_GAIT_STEP);
			} else {
				send_nack(disp, frame->msg_id, ERR_BUSY);
			}
		} else {
			send_nack(disp, frame->msg_id, ERR_BAD_LENGTH);
		}
		break;

	case CMD_SET_LEG_SITE:
		/* Cartesian kinematics offloaded to PC compiler */
		send_nack(disp, frame->msg_id, ERR_UNKNOWN_CMD);
		break;

	case CMD_SET_LEG_ANGLES:
		if (frame->length >= (uint8_t)PAYLOAD_LEN_SET_LEG_ANGLES) {
			uint8_t const leg = frame->payload[0];
			int16_t const coxa = (int16_t)frame->payload[1] * (int16_t)SCALE_FACTOR_TENTHS;
			int16_t const femur = (int16_t)frame->payload[2] * (int16_t)SCALE_FACTOR_TENTHS;
			int16_t const tibia = (int16_t)frame->payload[3] * (int16_t)SCALE_FACTOR_TENTHS;
			if (motion_planner_set_joint_target(
			        disp->planner, leg, coxa, femur, tibia, (uint16_t)DEFAULT_DIRECT_MOVE_MS
			    )) {
				send_ack(disp, (uint8_t)CMD_SET_LEG_ANGLES);
			} else {
				send_nack(disp, frame->msg_id, ERR_INVALID_PARAM);
			}
		} else {
			send_nack(disp, frame->msg_id, ERR_BAD_LENGTH);
		}
		break;

	case CMD_QUEUE_KEYFRAME:
		if (frame->length >= (uint8_t)PAYLOAD_LEN_QUEUE_KEYFRAME) {
			motion_keyframe_t kf;
			kf.leg_index = frame->payload[0];
			kf.target_coxa = (int16_t)(((uint16_t)frame->payload[1] << SHIFT_BYTE_8) | (uint16_t)frame->payload[2]);
			kf.target_femur = (int16_t)(((uint16_t)frame->payload[3] << SHIFT_BYTE_8) | (uint16_t)frame->payload[4]);
			kf.target_tibia = (int16_t)(((uint16_t)frame->payload[5] << SHIFT_BYTE_8) | (uint16_t)frame->payload[6]);
			kf.duration_ms = (uint16_t)(((uint16_t)frame->payload[7] << SHIFT_BYTE_8) | (uint16_t)frame->payload[8]);
			kf.flags = 0U;

			if (motion_planner_enqueue_keyframe(disp->planner, &kf)) {
				send_ack(disp, (uint8_t)CMD_QUEUE_KEYFRAME);
			} else {
				send_nack(disp, frame->msg_id, ERR_POOL_EXHAUSTED);
			}
		} else {
			send_nack(disp, frame->msg_id, ERR_BAD_LENGTH);
		}
		break;

	case CMD_CLEAR_QUEUE:
		(void)motion_planner_clear_queue(disp->planner);
		send_ack(disp, (uint8_t)CMD_CLEAR_QUEUE);
		break;

	case CMD_GET_STATUS: {
		uint8_t status_payload[PAYLOAD_LEN_STATUS];
		status_payload[0] = (uint8_t)gait_controller_get_state(disp->gait);
		status_payload[1] = (uint8_t)gait_controller_get_active_gait(disp->gait);
		status_payload[2] = motion_planner_queue_count(disp->planner);
		status_payload[3] = mem_pool_available(disp->pool);
		status_payload[4] = (uint8_t)disp->last_error;
		status_payload[5] = servo_is_attached(disp->planner->servo) ? 1U : 0U;
		status_payload[6] = disp->planner->speed_multiplier;
		command_dispatcher_send_response(disp, RESP_STATUS, status_payload, (uint8_t)PAYLOAD_LEN_STATUS);
		break;
	}

	case CMD_RESET_ERRORS:
		disp->last_error = ERR_NONE;
		send_ack(disp, (uint8_t)CMD_RESET_ERRORS);
		break;

	case CMD_GET_IMU: {
		imu_data_t imu_data;
		if ((disp->imu != (const imu_driver_t *)0) && imu_read_data(disp->imu, &imu_data)) {
			uint8_t imu_payload[PAYLOAD_LEN_IMU];
			imu_payload[0] = (uint8_t)((uint16_t)imu_data.roll >> SHIFT_BYTE_8);
			imu_payload[1] = (uint8_t)((uint16_t)imu_data.roll & 0xFFU);
			imu_payload[2] = (uint8_t)((uint16_t)imu_data.pitch >> SHIFT_BYTE_8);
			imu_payload[3] = (uint8_t)((uint16_t)imu_data.pitch & 0xFFU);
			imu_payload[4] = (uint8_t)((uint16_t)imu_data.accel_x >> SHIFT_BYTE_8);
			imu_payload[5] = (uint8_t)((uint16_t)imu_data.accel_x & 0xFFU);
			imu_payload[6] = (uint8_t)((uint16_t)imu_data.accel_y >> SHIFT_BYTE_8);
			imu_payload[7] = (uint8_t)((uint16_t)imu_data.accel_y & 0xFFU);
			imu_payload[8] = (uint8_t)((uint16_t)imu_data.accel_z >> SHIFT_BYTE_8);
			imu_payload[9] = (uint8_t)((uint16_t)imu_data.accel_z & 0xFFU);
			imu_payload[10] = (uint8_t)((uint16_t)imu_data.gyro_z >> SHIFT_BYTE_8);
			imu_payload[11] = (uint8_t)((uint16_t)imu_data.gyro_z & 0xFFU);
			command_dispatcher_send_response(disp, RESP_IMU, imu_payload, (uint8_t)PAYLOAD_LEN_IMU);
		} else {
			send_nack(disp, frame->msg_id, ERR_UNKNOWN_CMD);
		}
		break;
	}

	default:
		send_nack(disp, frame->msg_id, ERR_UNKNOWN_CMD);
		break;
	}
}
