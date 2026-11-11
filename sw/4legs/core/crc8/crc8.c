/**
 * @file crc8.c
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
 * @brief Bitwise CRC-8 checksum calculator implementation.
 *
 * Uses the ATM/SMBus generator polynomial x^8 + x^2 + x^1 + x^0 (0x07)
 * to provide robust frame validation with low flash and zero SRAM usage.
 */

#include "crc8.h"

enum {
	CRC8_POLYNOMIAL = 0x07, /**< ATM polynomial (x^8 + x^2 + x^1 + 1). */
	CRC8_INIT_VALUE = 0x00, /**< Initial CRC accumulator seed value. */
	CRC8_MSB_MASK = 0x80,   /**< MSB mask for 8-bit CRC byte. */
	CRC8_BITS_PER_BYTE = 8  /**< Number of bits processed per byte loop. */
};

uint8_t crc8_update(uint8_t crc, uint8_t byte) {
	crc ^= byte;

	for (uint8_t bit = 0; bit < (uint8_t)CRC8_BITS_PER_BYTE; ++bit) {
		if ((crc & (uint8_t)CRC8_MSB_MASK) != 0U) {
			uint8_t const shifted = (uint8_t)(crc << 1U);
			crc = (uint8_t)(shifted ^ (uint8_t)CRC8_POLYNOMIAL);
		} else {
			crc = (uint8_t)(crc << 1U);
		}
	}

	return crc;
}

uint8_t crc8_calculate(const uint8_t *data, uint8_t length) {
	if (data == (const uint8_t *)0) {
		return (uint8_t)CRC8_INIT_VALUE;
	}

	uint8_t crc = (uint8_t)CRC8_INIT_VALUE;

	for (uint8_t i = 0; i < length; ++i) {
		crc = crc8_update(crc, data[i]);
	}

	return crc;
}
