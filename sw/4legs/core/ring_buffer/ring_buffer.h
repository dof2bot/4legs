/**
 * @file ring_buffer.h
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
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) circular FIFO ring buffer.
 *
 * Designed specifically for 8-bit AVR microcontrollers:
 *  - 8-bit head and tail indexes guarantee atomic read/write without disabling interrupts.
 *  - Enforces power-of-two capacity for 1-cycle bitwise masking (& mask) instead of
 *    expensive multi-cycle software division.
 *  - Capacity is (size - 1) bytes to unambiguously distinguish empty from full states.
 *  - Fully compliant with MISRA-C guidelines (strict bounds checks, no dynamic allocation).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Circular ring buffer descriptor structure.
 */
typedef struct {
	uint8_t *storage;      /**< Pointer to caller-allocated backing memory array. */
	uint8_t size;          /**< Total buffer capacity in bytes (must be a power of 2). */
	volatile uint8_t head; /**< Write pointer index modified by producer / ISR. */
	volatile uint8_t tail; /**< Read pointer index modified by consumer. */
} ring_buffer_t;

/**
 * @brief Initialize ring buffer with caller-provided backing memory.
 *
 * Enforces power-of-two capacity for high-performance bitwise wrapping on AVR.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @param storage Pointer to pre-allocated uint8_t array.
 * @param size Total capacity in bytes (MUST be a power of 2: 16, 32, 64, 128).
 * @return bool true on success, false if parameters are invalid or size is not power-of-2.
 */
bool ring_buffer_init(ring_buffer_t *rb, uint8_t *storage, uint8_t size);

/**
 * @brief Push a single byte into the ring buffer.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @param byte Data byte to enqueue.
 * @return bool true if enqueued successfully, false if buffer is full.
 */
bool ring_buffer_push(ring_buffer_t *rb, uint8_t byte);

/**
 * @brief Pop a single byte from the ring buffer.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @param byte Output pointer where popped byte is stored (may be NULL to discard).
 * @return bool true if dequeued successfully, false if buffer is empty.
 */
bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *byte);

/**
 * @brief Peek at the next byte without removing it from the buffer.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @param byte Output pointer where peeked byte is stored.
 * @return bool true if byte read, false if buffer is empty.
 */
bool ring_buffer_peek(const ring_buffer_t *rb, uint8_t *byte);

/**
 * @brief Check if the ring buffer is empty.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @return bool true if empty (or rb is NULL), false otherwise.
 */
bool ring_buffer_is_empty(const ring_buffer_t *rb);

/**
 * @brief Check if the ring buffer is full.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @return bool true if full (or rb is NULL), false otherwise.
 */
bool ring_buffer_is_full(const ring_buffer_t *rb);

/**
 * @brief Get the number of elements currently stored in the buffer.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @return uint8_t Number of bytes available.
 */
uint8_t ring_buffer_count(const ring_buffer_t *rb);

/**
 * @brief Reset / clear the ring buffer to empty state.
 *
 * @param rb Pointer to ring_buffer_t instance.
 * @return bool true on success, false if rb is NULL.
 */
bool ring_buffer_clear(ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif
