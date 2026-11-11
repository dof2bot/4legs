/**
 * @file protocol_types.h
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
 * @brief Binary serial communication protocol definitions and message frames.
 *
 * Binary Frame Anatomy:
 *  +--------+--------+--------+--------------------------+--------+
 *  |  SYNC  | MSG_ID | LENGTH |         PAYLOAD          | CRC-8  |
 *  | (0xAA) | (1 B)  | (1 B)  |        (0..16 B)         | (1 B)  |
 *  +--------+--------+--------+--------------------------+--------+
 *           |<--------------- CRC-8 Scope -------------->|
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	PROTOCOL_SYNC_BYTE = 0xAA, /**< Frame start synchronization delimiter. */
	FRAME_OFFSET_SYNC = 0,
	FRAME_OFFSET_MSG_ID = 1,
	FRAME_OFFSET_LEN = 2,
	FRAME_OFFSET_PAYLOAD = 3,
	FRAME_HEADER_SIZE = 3,
	FRAME_CRC_SIZE = 1,
	FRAME_OVERHEAD_SIZE = 4, /**< HEADER + CRC */
	FRAME_PAYLOAD_MAX = 16,  /**< Maximum payload length per frame in bytes. */
	SHIFT_BYTE_8 = 8U        /**< Bit shift for 16-bit high/low byte packing. */
};

/**
 * @brief Canonical payload lengths for commands and responses in bytes.
 */
typedef enum {
	PAYLOAD_LEN_PING = 0,
	PAYLOAD_LEN_STOP = 0,
	PAYLOAD_LEN_SET_SPEED = 1,
	PAYLOAD_LEN_GAIT_STEP = 2,
	PAYLOAD_LEN_SET_LEG_SITE = 7,
	PAYLOAD_LEN_SET_LEG_ANGLES = 4,
	PAYLOAD_LEN_QUEUE_KEYFRAME = 9,
	PAYLOAD_LEN_CLEAR_QUEUE = 0,
	PAYLOAD_LEN_GET_STATUS = 0,
	PAYLOAD_LEN_RESET_ERRORS = 0,
	PAYLOAD_LEN_GET_IMU = 0,
	PAYLOAD_LEN_ACK = 1,
	PAYLOAD_LEN_NACK = 2,
	PAYLOAD_LEN_PONG = 0,
	PAYLOAD_LEN_STATUS = 7,
	PAYLOAD_LEN_IMU = 12
} protocol_payload_len_t;

/**
 * @brief Inbound binary protocol command IDs (PC -> Robot, 0x01..0x7F).
 */
typedef enum {
	CMD_PING = 0x01,           /**< Heartbeat ping (0 B payload). */
	CMD_STOP = 0x02,           /**< Emergency stop / abort motion (0 B). */
	CMD_SET_SPEED = 0x03,      /**< Set speed multiplier (1 B: 1..10). */
	CMD_GAIT_STEP = 0x10,      /**< Execute gait (2 B: gait_id, steps). */
	CMD_SET_LEG_SITE = 0x11,   /**< Direct Cartesian leg site (7 B: leg, x_hi, x_lo, y_hi, y_lo, z_hi, z_lo). */
	CMD_SET_LEG_ANGLES = 0x12, /**< Direct joint angles (4 B: leg, alpha, beta, gamma). */
	CMD_QUEUE_KEYFRAME = 0x20, /**< Enqueue trajectory keyframe (9 B: leg, x, y, z, duration_ms). */
	CMD_CLEAR_QUEUE = 0x21,    /**< Clear trajectory queue (0 B). */
	CMD_GET_STATUS = 0x30,     /**< Request status frame (0 B). */
	CMD_RESET_ERRORS = 0x31,   /**< Clear error flags (0 B). */
	CMD_GET_IMU = 0x32         /**< Request 6-DOF IMU telemetry (0 B). */
} protocol_cmd_id_t;

/**
 * @brief Outbound binary protocol response IDs (Robot -> PC, 0x80..0xFF).
 */
typedef enum {
	RESP_ACK = 0x80,   /**< Positive acknowledgment (1 B: cmd_id). */
	RESP_NACK = 0x81,  /**< Negative rejection (2 B: cmd_id, err_code). */
	RESP_PONG = 0x84,  /**< Pong heartbeat response (0 B). */
	RESP_STATUS = 0x90, /**< Telemetry status packet (7 B). */
	RESP_IMU = 0x91    /**< 6-DOF IMU attitude and acceleration telemetry (12 B). */
} protocol_resp_id_t;

/**
 * @brief Binary protocol error and rejection reason codes.
 */
typedef enum {
	ERR_NONE = 0x00,
	ERR_BAD_CRC = 0x01,
	ERR_BAD_LENGTH = 0x02,
	ERR_UNKNOWN_CMD = 0x03,
	ERR_INVALID_PARAM = 0x04,
	ERR_POOL_EXHAUSTED = 0x05,
	ERR_QUEUE_FULL = 0x06,
	ERR_KINEMATIC_LIMIT = 0x07,
	ERR_BUSY = 0x08
} protocol_err_code_t;

#ifdef __cplusplus
}
#endif
