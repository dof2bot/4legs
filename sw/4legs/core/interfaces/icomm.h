/**
 * @file icomm.h
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
 * @brief Abstract bidirectional communication stream interface (Output/Input Port / DIP).
 */

#pragma once

#include "progmem_compat.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct comm_stream comm_stream_t;

/**
 * @brief Bidirectional communication stream polymorphic object encapsulation.
 */
struct comm_stream {
	void *context;                                             /**< Opaque context pointer. */
	bool (*available)(void *context);                          /**< Check for available bytes. */
	uint8_t (*read)(void *context);                            /**< Read single byte. */
	bool (*write)(void *context, uint8_t byte);                /**< Transmit single byte. */
	bool (*write_str)(void *context, const char *str);         /**< Transmit RAM string. */
	bool (*write_str_p)(void *context, const char *flash_str); /**< Transmit Flash (PROGMEM) string. */
};

static inline bool comm_available(const comm_stream_t *stream) {
	if ((stream != (const comm_stream_t *)0) && (stream->available != (bool (*)(void *))0)) {
		return stream->available(stream->context);
	}
	return false;
}

static inline uint8_t comm_read(const comm_stream_t *stream) {
	if ((stream != (const comm_stream_t *)0) && (stream->read != (uint8_t(*)(void *))0)) {
		return stream->read(stream->context);
	}
	return 0U;
}

static inline bool comm_write(const comm_stream_t *stream, uint8_t byte) {
	if ((stream != (const comm_stream_t *)0) && (stream->write != (bool (*)(void *, uint8_t))0)) {
		return stream->write(stream->context, byte);
	}
	return false;
}

static inline bool comm_write_str(const comm_stream_t *stream, const char *str) {
	if ((stream != (const comm_stream_t *)0) && (stream->write_str != (bool (*)(void *, const char *))0)) {
		return stream->write_str(stream->context, str);
	}
	return false;
}

static inline bool comm_write_str_p(const comm_stream_t *stream, const char *flash_str) {
	if ((stream != (const comm_stream_t *)0) && (stream->write_str_p != (bool (*)(void *, const char *))0)) {
		return stream->write_str_p(stream->context, flash_str);
	}
	return false;
}

#ifdef __cplusplus
}
#endif
