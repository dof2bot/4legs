/**
 * @file mem_pool.c
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
 * @brief MISRA-C compliant fixed-size block memory pool allocator implementation.
 */

#include "mem_pool.h"

bool mem_pool_init(mem_pool_t *pool, uint8_t *storage, uint8_t *free_stack, uint8_t block_size, uint8_t block_count) {
	if ((pool == (mem_pool_t *)0) || (storage == (uint8_t *)0) || (free_stack == (uint8_t *)0) || (block_size == 0U) ||
	    (block_count == 0U)) {
		return false;
	}

	pool->storage = storage;
	pool->free_stack = free_stack;
	pool->block_size = block_size;
	pool->block_count = block_count;
	pool->free_count = block_count;

	/* Initialize free stack with indices [0 .. block_count - 1] */
	for (uint8_t i = 0U; i < block_count; ++i) {
		pool->free_stack[i] = i;
	}

	return true;
}

void *mem_pool_alloc(mem_pool_t *pool) {
	if (pool == (mem_pool_t *)0) {
		return (void *)0;
	}

	if (pool->free_count == 0U) {
		return (void *)0; /* Pool exhausted */
	}

	pool->free_count--;
	uint8_t const block_index = pool->free_stack[pool->free_count];
	uint16_t const byte_offset = (uint16_t)block_index * (uint16_t)pool->block_size;

	return (void *)(&pool->storage[byte_offset]);
}

bool mem_pool_free(mem_pool_t *pool, const void *block) {
	if ((pool == (mem_pool_t *)0) || (block == (const void *)0)) {
		return false;
	}

	const uint8_t *const ptr = (const uint8_t *)block;
	const uint8_t *const start = pool->storage;
	uint16_t const total_bytes = (uint16_t)pool->block_size * (uint16_t)pool->block_count;

	/* Bounds check */
	if ((ptr < start) || (ptr >= &start[total_bytes])) {
		return false;
	}

	/* Alignment check */
	uint16_t const offset = (uint16_t)(ptr - start);
	if ((offset % (uint16_t)pool->block_size) != 0U) {
		return false;
	}

	uint8_t const block_index = (uint8_t)(offset / (uint16_t)pool->block_size);

	/* Pool cannot exceed capacity */
	if (pool->free_count >= pool->block_count) {
		return false;
	}

	/* Defensive double-free verification */
	for (uint8_t i = 0U; i < pool->free_count; ++i) {
		if (pool->free_stack[i] == block_index) {
			return false; /* Double-free detected */
		}
	}

	pool->free_stack[pool->free_count] = block_index;
	pool->free_count++;

	return true;
}

uint8_t mem_pool_available(const mem_pool_t *pool) {
	if (pool == (const mem_pool_t *)0) {
		return 0U;
	}

	return pool->free_count;
}

bool mem_pool_is_full(const mem_pool_t *pool) {
	if (pool == (const mem_pool_t *)0) {
		return true;
	}

	return (pool->free_count == 0U);
}

bool mem_pool_is_empty(const mem_pool_t *pool) {
	if (pool == (const mem_pool_t *)0) {
		return true;
	}

	return (pool->free_count == pool->block_count);
}
