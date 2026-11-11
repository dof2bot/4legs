/**
 * @file ring_buffer.c
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
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) circular FIFO ring buffer implementation.
 */

#include "ring_buffer.h"

/**
 * @brief Checks if a given 8-bit unsigned integer is a power of two.
 *
 * @param value Value to check.
 * @return bool true if power of two, false otherwise.
 */
static inline bool is_power_of_two(uint8_t value) { return ((value != 0U) && ((value & (uint8_t)(value - 1U)) == 0U)); }

bool ring_buffer_init(ring_buffer_t *rb, uint8_t *storage, uint8_t size) {
	if ((rb == (ring_buffer_t *)0) || (storage == (uint8_t *)0) || (!is_power_of_two(size))) {
		return false;
	}

	rb->storage = storage;
	rb->size = size;
	rb->head = 0U;
	rb->tail = 0U;

	return true;
}

bool ring_buffer_push(ring_buffer_t *rb, uint8_t byte) {
	if (rb == (ring_buffer_t *)0) {
		return false;
	}

	uint8_t const next_head = (uint8_t)((rb->head + 1U) & (uint8_t)(rb->size - 1U));

	if (next_head == rb->tail) {
		return false; /* Buffer is full */
	}

	rb->storage[rb->head] = byte;
	rb->head = next_head;

	return true;
}

bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *byte) {
	if (rb == (ring_buffer_t *)0) {
		return false;
	}

	if (rb->head == rb->tail) {
		return false; /* Buffer is empty */
	}

	if (byte != (uint8_t *)0) {
		*byte = rb->storage[rb->tail];
	}

	rb->tail = (uint8_t)((rb->tail + 1U) & (uint8_t)(rb->size - 1U));

	return true;
}

bool ring_buffer_peek(const ring_buffer_t *rb, uint8_t *byte) {
	if ((rb == (const ring_buffer_t *)0) || (byte == (uint8_t *)0)) {
		return false;
	}

	if (rb->head == rb->tail) {
		return false; /* Buffer is empty */
	}

	*byte = rb->storage[rb->tail];

	return true;
}

bool ring_buffer_is_empty(const ring_buffer_t *rb) {
	if (rb == (const ring_buffer_t *)0) {
		return true;
	}

	return (rb->head == rb->tail);
}

bool ring_buffer_is_full(const ring_buffer_t *rb) {
	if (rb == (const ring_buffer_t *)0) {
		return true;
	}

	uint8_t const next_head = (uint8_t)((rb->head + 1U) & (uint8_t)(rb->size - 1U));
	return (next_head == rb->tail);
}

uint8_t ring_buffer_count(const ring_buffer_t *rb) {
	if (rb == (const ring_buffer_t *)0) {
		return 0U;
	}

	return (uint8_t)((rb->head - rb->tail) & (uint8_t)(rb->size - 1U));
}

bool ring_buffer_clear(ring_buffer_t *rb) {
	if (rb == (ring_buffer_t *)0) {
		return false;
	}

	rb->head = 0U;
	rb->tail = 0U;

	return true;
}
