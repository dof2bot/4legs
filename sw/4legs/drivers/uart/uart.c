/**
 * @file uart.c
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
 * @brief Hardware USART0 asynchronous serial transceiver with interrupt ring buffers.
 */

#include "uart.h"

#include "ring_buffer.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

enum {
	UBRR_SHIFT_HIGH_BYTE = 8U,
	UBRR_LOW_BYTE_MASK = 0xFFU,
	UBRR_DIVISOR_FACTOR_U2X = 8UL
};

static uint8_t s_rx_storage[UART_RX_BUFFER_SIZE];
static ring_buffer_t s_rx_ring;
static uint8_t s_tx_storage[UART_TX_BUFFER_SIZE];
static ring_buffer_t s_tx_ring;

ISR(USART_RX_vect) { (void)ring_buffer_push(&s_rx_ring, UDR0); }

ISR(USART_UDRE_vect) {
	uint8_t byte = 0U;
	if (ring_buffer_pop(&s_tx_ring, &byte)) {
		UDR0 = byte;
	} else {
		/* No more data to send: disable UDRE interrupt */
		UCSR0B &= (uint8_t) ~(1U << UDRIE0);
	}
}

bool uart_init(uint32_t baud_rate) {
	if (baud_rate == 0UL) {
		return false;
	}

	if (!ring_buffer_init(&s_rx_ring, s_rx_storage, (uint8_t)UART_RX_BUFFER_SIZE)) {
		return false;
	}

	if (!ring_buffer_init(&s_tx_ring, s_tx_storage, (uint8_t)UART_TX_BUFFER_SIZE)) {
		return false;
	}

	/* Enable 2X speed mode (U2X0) for reduced baud rate error at 57600/115200 */
	UCSR0A |= (1U << U2X0);

	uint16_t const ubrr_val = (uint16_t)(((F_CPU / ((uint32_t)UBRR_DIVISOR_FACTOR_U2X * baud_rate)) + 1UL) / 2UL - 1UL);
	UBRR0H = (uint8_t)(ubrr_val >> (uint8_t)UBRR_SHIFT_HIGH_BYTE);
	UBRR0L = (uint8_t)(ubrr_val & (uint8_t)UBRR_LOW_BYTE_MASK);

	/* Set frame format: 8 data bits, 1 stop bit, no parity (8N1) */
	UCSR0C = (uint8_t)((1U << UCSZ01) | (1U << UCSZ00));

	/* Enable receiver, transmitter, and receive interrupt */
	UCSR0B = (uint8_t)((1U << RXEN0) | (1U << TXEN0) | (1U << RXCIE0));

	return true;
}

bool uart_available(void) { return !ring_buffer_is_empty(&s_rx_ring); }

uint8_t uart_read(void) {
	uint8_t byte = 0U;
	(void)ring_buffer_pop(&s_rx_ring, &byte);
	return byte;
}

bool uart_write(uint8_t byte) {
	/* Push into TX ring buffer */
	if (!ring_buffer_push(&s_tx_ring, byte)) {
		return false;
	}

	/* Enable UDRE interrupt so hardware ISR starts draining the buffer */
	UCSR0B |= (uint8_t)(1U << UDRIE0);
	return true;
}

bool uart_write_str(const char *str) {
	if (str == (const char *)0) {
		return false;
	}

	while (*str != '\0') {
		if (!uart_write((uint8_t)(*str))) {
			return false;
		}
		str++;
	}

	return true;
}

bool uart_write_str_P(const char *flash_str) {
	if (flash_str == (const char *)0) {
		return false;
	}

	char c = (char)pgm_read_byte(flash_str);
	while (c != '\0') {
		if (!uart_write((uint8_t)c)) {
			return false;
		}
		flash_str++;
		c = (char)pgm_read_byte(flash_str);
	}

	return true;
}

bool uart_flush_tx(void) {
	while (!ring_buffer_is_empty(&s_tx_ring) || ((UCSR0A & (uint8_t)(1U << UDRE0)) == 0U)) {
		/* Wait for transmission to finish */
	}
	return true;
}

static bool uart_stream_available(void *context) {
	(void)context;
	return uart_available();
}

static uint8_t uart_stream_read(void *context) {
	(void)context;
	return uart_read();
}

static bool uart_stream_write(void *context, uint8_t byte) {
	(void)context;
	return uart_write(byte);
}

static bool uart_stream_write_str(void *context, const char *str) {
	(void)context;
	return uart_write_str(str);
}

static bool uart_stream_write_str_p(void *context, const char *flash_str) {
	(void)context;
	return uart_write_str_P(flash_str);
}

comm_stream_t uart_get_stream(void) {
	comm_stream_t stream;
	stream.context = (void *)0;
	stream.available = uart_stream_available;
	stream.read = uart_stream_read;
	stream.write = uart_stream_write;
	stream.write_str = uart_stream_write_str;
	stream.write_str_p = uart_stream_write_str_p;

	return stream;
}
