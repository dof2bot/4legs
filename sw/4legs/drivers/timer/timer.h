/**
 * @file timer.h
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
 * @brief 1ms system tick timebase generator using 8-bit Timer2 in CTC mode.
 *
 * CTC Frequency & Period Derivation:
 *   f_CPU = 16,000,000 Hz (16 MHz Crystal Oscillator)
 *   Prescaler N = 64 (CS22 = 1, CS21 = 0, CS20 = 0)
 *   Target Tick Frequency = 1,000 Hz (T = 1.0 ms)
 *   OCR2A = [ 16,000,000 / (64 * 1,000) ] - 1 = 249
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Timer2 in CTC mode with 1ms compare match interrupts.
 *
 * @return bool true on success.
 */
bool timer_init(void);

/**
 * @brief Retrieve current system uptime in milliseconds.
 *
 * @return uint32_t Uptime in milliseconds.
 */
uint32_t timer_millis(void);

/**
 * @brief Check if a 20ms (50 Hz) periodic tick event has occurred.
 *
 * Clears the flag upon reading.
 *
 * @return bool true if 20ms tick occurred.
 */
bool timer_check_20ms_tick(void);

#ifdef __cplusplus
}
#endif
