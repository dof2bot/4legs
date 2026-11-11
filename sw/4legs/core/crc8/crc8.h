/**
 * @file crc8.h
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
 * @brief Hardware-friendly CRC-8 checksum calculation using ATM/SMBus polynomial standard.
 *
 * Algorithm Specification:
 *  - Polynomial:         x^8 + x^2 + x^1 + x^0 = 0x07 (ATM / SMBus)
 *  - Initial Value:      0x00
 *  - Input Reflection:   False (MSB-first processing)
 *  - Output Reflection:  False
 *  - Final XOR Value:    0x00
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update running CRC-8 accumulator with a single data byte.
 *
 * @param crc Current CRC accumulator state.
 * @param byte Next incoming byte from communication stream.
 * @return uint8_t Updated CRC-8 accumulator value.
 */
uint8_t crc8_update(uint8_t crc, uint8_t byte);

/**
 * @brief Compute CRC-8 checksum over a contiguous memory buffer.
 *
 * @param data Pointer to input data buffer.
 * @param length Number of bytes to process.
 * @return uint8_t Final CRC-8 checksum value.
 */
uint8_t crc8_calculate(const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif
