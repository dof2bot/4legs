/**
 * @file mem_pool.h
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
 * @brief MISRA-C compliant fixed-size block memory pool allocator.
 *
 * Provides deterministic, O(1) constant-time allocation and deallocation of
 * fixed-size memory blocks from a statically allocated buffer.
 *
 * Architecture & Safety:
 *  - Enforces MISRA-C:2012 Rule 21.3 (no malloc/free from <stdlib.h>).
 *  - Zero external fragmentation.
 *  - Double-free detection and defensive bounds checking.
 *  - Statically verifiable upper bound on memory consumption.
 *
 * Topology:
 *
 *    Storage Buffer (block_size * block_count):
 *    +-----------+-----------+-----------+ ... +-----------+
 *    |  Block 0  |  Block 1  |  Block 2  |     | Block N-1 |
 *    +-----------+-----------+-----------+ ... +-----------+
 *
 *    Free Index Stack:
 *    [ idx_0, idx_1, idx_2, ... ] -> LIFO stack pointing to available blocks.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Memory pool descriptor structure.
 */
typedef struct {
	uint8_t *storage;    /**< Pointer to pre-allocated contiguous memory buffer. */
	uint8_t *free_stack; /**< Array storing indices [0..block_count-1] of free blocks. */
	uint8_t block_size;  /**< Size in bytes of each individual block. */
	uint8_t block_count; /**< Total capacity in number of blocks. */
	uint8_t free_count;  /**< Current number of free blocks available for allocation. */
} mem_pool_t;

/**
 * @brief Initializes a memory pool with caller-provided backing arrays.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @param storage Contiguous memory buffer of at least (block_size * block_count) bytes.
 * @param free_stack Array of at least block_count uint8_t elements.
 * @param block_size Size of each block in bytes (must be > 0).
 * @param block_count Total number of blocks (must be > 0).
 * @return bool true if initialized successfully, false on invalid arguments.
 */
bool mem_pool_init(mem_pool_t *pool, uint8_t *storage, uint8_t *free_stack, uint8_t block_size, uint8_t block_count);

/**
 * @brief Allocates one fixed-size block from the memory pool in O(1) time.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @return void* Pointer to the allocated block, or NULL ((void*)0) if pool is exhausted.
 */
void *mem_pool_alloc(mem_pool_t *pool);

/**
 * @brief Deallocates a previously allocated block back to the pool in O(1) time.
 *
 * Performs defensive address verification to ensure the block belongs to this pool
 * and aligns with block boundaries.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @param block Pointer to block previously returned by mem_pool_alloc.
 * @return bool true if freed successfully, false if block is invalid, out of bounds, or double-freed.
 */
bool mem_pool_free(mem_pool_t *pool, const void *block);

/**
 * @brief Returns the number of currently available free blocks in the pool.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @return uint8_t Number of free blocks.
 */
uint8_t mem_pool_available(const mem_pool_t *pool);

/**
 * @brief Checks if the memory pool is completely exhausted.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @return bool true if full (0 free blocks remaining), false otherwise.
 */
bool mem_pool_is_full(const mem_pool_t *pool);

/**
 * @brief Checks if all blocks in the memory pool are free.
 *
 * @param pool Pointer to mem_pool_t instance.
 * @return bool true if all blocks free, false otherwise.
 */
bool mem_pool_is_empty(const mem_pool_t *pool);

#ifdef __cplusplus
}
#endif
