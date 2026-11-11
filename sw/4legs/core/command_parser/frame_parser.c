/**
 * @file frame_parser.c
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
 * @brief Streaming non-blocking binary protocol frame parser state machine implementation.
 */

#include "frame_parser.h"

void frame_parser_init(frame_parser_t *parser) {
	if (parser != (frame_parser_t *)0) {
		parser->state = PARSER_STATE_WAIT_SYNC;
		parser->frame.msg_id = 0U;
		parser->frame.length = 0U;
		parser->bytes_read = 0U;
		parser->running_crc = 0U;
	}
}

bool frame_parser_feed_byte(frame_parser_t *parser, uint8_t byte, protocol_frame_t *out_frame) {
	if ((parser == (frame_parser_t *)0) || (out_frame == (protocol_frame_t *)0)) {
		return false;
	}

	bool frame_ready = false;

	switch (parser->state) {
	case PARSER_STATE_WAIT_SYNC:
		if (byte == (uint8_t)PROTOCOL_SYNC_BYTE) {
			parser->state = PARSER_STATE_READ_MSG_ID;
			parser->running_crc = 0U;
		}
		break;

	case PARSER_STATE_READ_MSG_ID:
		parser->frame.msg_id = byte;
		parser->running_crc = crc8_update(parser->running_crc, byte);
		parser->state = PARSER_STATE_READ_LEN;
		break;

	case PARSER_STATE_READ_LEN:
		if (byte <= (uint8_t)FRAME_PAYLOAD_MAX) {
			parser->frame.length = byte;
			parser->running_crc = crc8_update(parser->running_crc, byte);
			parser->bytes_read = 0U;
			if (byte == 0U) {
				parser->state = PARSER_STATE_VERIFY_CRC;
			} else {
				parser->state = PARSER_STATE_READ_PAYLOAD;
			}
		} else {
			/* Length exceeds maximum allowed: resynchronize */
			parser->state = PARSER_STATE_WAIT_SYNC;
		}
		break;

	case PARSER_STATE_READ_PAYLOAD:
		parser->frame.payload[parser->bytes_read] = byte;
		parser->running_crc = crc8_update(parser->running_crc, byte);
		parser->bytes_read++;
		if (parser->bytes_read >= parser->frame.length) {
			parser->state = PARSER_STATE_VERIFY_CRC;
		}
		break;

	case PARSER_STATE_VERIFY_CRC:
		if (byte == parser->running_crc) {
			*out_frame = parser->frame;
			frame_ready = true;
		}
		parser->state = PARSER_STATE_WAIT_SYNC;
		break;

	default:
		parser->state = PARSER_STATE_WAIT_SYNC;
		break;
	}

	return frame_ready;
}
