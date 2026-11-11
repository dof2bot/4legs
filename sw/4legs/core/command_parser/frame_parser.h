/**
 * @file frame_parser.h
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
 * @brief Streaming non-blocking binary protocol frame parser state machine.
 */

#pragma once

#include "crc8.h"
#include "icomm.h"
#include "protocol_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PARSER_STATE_WAIT_SYNC = 0,
	PARSER_STATE_READ_MSG_ID = 1,
	PARSER_STATE_READ_LEN = 2,
	PARSER_STATE_READ_PAYLOAD = 3,
	PARSER_STATE_VERIFY_CRC = 4
} parser_state_t;

typedef struct {
	uint8_t msg_id;
	uint8_t length;
	uint8_t payload[FRAME_PAYLOAD_MAX];
} protocol_frame_t;

typedef struct {
	parser_state_t state;
	protocol_frame_t frame;
	uint8_t bytes_read;
	uint8_t running_crc;
} frame_parser_t;

/**
 * @brief Initializes frame parser state machine.
 *
 * @param parser Pointer to frame_parser_t.
 */
void frame_parser_init(frame_parser_t *parser);

/**
 * @brief Feeds a single byte from stream into parser state machine.
 *
 * @param parser Pointer to frame_parser_t.
 * @param byte Incoming byte.
 * @param out_frame Pointer to protocol_frame_t filled when complete frame validated.
 * @return bool true if a complete and valid frame was parsed, false otherwise.
 */
bool frame_parser_feed_byte(frame_parser_t *parser, uint8_t byte, protocol_frame_t *out_frame);

#ifdef __cplusplus
}
#endif
