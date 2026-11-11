/**
 * @file timer.c
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
 * @brief 1ms system tick timebase generator implementation.
 */

#include "timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>

enum {
	TIMER2_OCR2A_1MS = 249U,    /**< 16MHz / (64 * 1000Hz) - 1 = 249 for 1ms CTC tick. */
	TIMER2_TICKS_PER_20MS = 20U /**< 20 x 1ms ticks for 50Hz motion clock. */
};

static volatile uint32_t s_system_millis = 0UL;
static volatile bool s_flag_20ms = false;
static uint8_t s_counter_20ms = 0U;

ISR(TIMER2_COMPA_vect) {
	s_system_millis++;
	s_counter_20ms++;
	if (s_counter_20ms >= (uint8_t)TIMER2_TICKS_PER_20MS) {
		s_counter_20ms = 0U;
		s_flag_20ms = true;
	}
}

bool timer_init(void) {
	/* Configure Timer2 for CTC Mode (Clear Timer on Compare Match) */
	TCCR2A = (1U << WGM21);
	TCCR2B = (1U << CS22); /* Prescaler 64 */
	TCNT2 = 0U;
	OCR2A = (uint8_t)TIMER2_OCR2A_1MS;
	TIMSK2 |= (1U << OCIE2A);
	return true;
}

uint32_t timer_millis(void) {
	uint8_t const sreg = SREG;
	cli();
	uint32_t const ms = s_system_millis;
	SREG = sreg;
	return ms;
}

bool timer_check_20ms_tick(void) {
	uint8_t const sreg = SREG;
	cli();
	bool const tick = s_flag_20ms;
	s_flag_20ms = false;
	SREG = sreg;
	return tick;
}
