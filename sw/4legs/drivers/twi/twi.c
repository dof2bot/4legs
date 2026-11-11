/**
 * @file twi.c
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
 * @brief Implementation of hardware Two-Wire Interface (TWI/I2C) master driver.
 */

#include "twi.h"

#include <avr/io.h>
#include <util/twi.h>

void twi_init(twi_bitrate_t bitrate) {
	/* Enable internal pull-ups on SDA (PC4) and SCL (PC5) */
	PORTC |= (uint8_t)((1U << PORTC4) | (1U << PORTC5));
	DDRC &= (uint8_t) ~((1U << DDC4) | (1U << DDC5));

	/* Prescaler = 1 (TWPS1 = 0, TWPS0 = 0) */
	TWSR = (uint8_t)TWI_PRESCALER_1;

	/* Set TWI Bit Rate Register */
	TWBR = (uint8_t)bitrate;

	/* Enable TWI peripheral */
	TWCR = (uint8_t)(1U << TWEN);
}

twi_result_t twi_start(uint8_t slave_addr, twi_op_mode_t mode) {
	/* Transmit START condition */
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWSTA) | (1U << TWEN));

	/* Bounded wait for transmission completion */
	uint16_t timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWINT)) == 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	/* Verify START or Repeated START status */
	uint8_t const start_status = (uint8_t)(TWSR & (uint8_t)TWI_STATUS_MASK);
	if ((start_status != (uint8_t)TW_START) &&
	    (start_status != (uint8_t)TW_REP_START)) {
		return TWI_ERROR_START;
	}

	/* Construct SLA+W or SLA+R byte */
	uint8_t const sla_addr = (uint8_t)(slave_addr << 1U);
	uint8_t const r_bit =
		(mode == TWI_READ) ? (uint8_t)TWI_SLA_READ_BIT : 0U;
	uint8_t const sla = (uint8_t)(sla_addr | r_bit);
	TWDR = sla;
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN));

	/* Bounded wait for address transmission */
	timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWINT)) == 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	/* Verify SLA ACK status */
	uint8_t const sla_status = (uint8_t)(TWSR & (uint8_t)TWI_STATUS_MASK);
	if ((mode == TWI_WRITE) && (sla_status != (uint8_t)TW_MT_SLA_ACK)) {
		return TWI_ERROR_SLA_NACK;
	}
	if ((mode == TWI_READ) && (sla_status != (uint8_t)TW_MR_SLA_ACK)) {
		return TWI_ERROR_SLA_NACK;
	}

	return TWI_SUCCESS;
}

twi_result_t twi_write(uint8_t data) {
	TWDR = data;
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN));

	/* Bounded wait for data transmission */
	uint16_t timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWINT)) == 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	/* Verify data byte ACK status */
	uint8_t const data_status = (uint8_t)(TWSR & (uint8_t)TWI_STATUS_MASK);
	if (data_status != (uint8_t)TW_MT_DATA_ACK) {
		return TWI_ERROR_DATA_NACK;
	}

	return TWI_SUCCESS;
}

twi_result_t twi_read_ack(uint8_t *data) {
	if (data == (uint8_t *)0) {
		return TWI_ERROR_PARAM;
	}

	/* Request byte with ACK generation */
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN) | (1U << TWEA));

	/* Bounded wait for data reception */
	uint16_t timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWINT)) == 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	/* Verify data byte ACK received status */
	uint8_t const status = (uint8_t)(TWSR & (uint8_t)TWI_STATUS_MASK);
	if (status != (uint8_t)TW_MR_DATA_ACK) {
		return TWI_ERROR_DATA_NACK;
	}

	*data = TWDR;
	return TWI_SUCCESS;
}

twi_result_t twi_read_nack(uint8_t *data) {
	if (data == (uint8_t *)0) {
		return TWI_ERROR_PARAM;
	}

	/* Request byte with NACK generation (final byte of reception) */
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN));

	/* Bounded wait for data reception */
	uint16_t timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWINT)) == 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	/* Verify data byte NACK received status */
	uint8_t const status = (uint8_t)(TWSR & (uint8_t)TWI_STATUS_MASK);
	if (status != (uint8_t)TW_MR_DATA_NACK) {
		return TWI_ERROR_DATA_NACK;
	}

	*data = TWDR;
	return TWI_SUCCESS;
}

twi_result_t twi_stop(void) {
	/* Transmit STOP condition */
	TWCR = (uint8_t)((1U << TWINT) | (1U << TWSTO) | (1U << TWEN));

	/* Bounded wait until hardware clears TWSTO flag */
	uint16_t timeout = (uint16_t)TWI_TIMEOUT_CYCLES;
	while ((timeout > 0U) && ((TWCR & (uint8_t)(1U << TWSTO)) != 0U)) {
		timeout--;
	}
	if (timeout == 0U) {
		return TWI_ERROR_TIMEOUT;
	}

	return TWI_SUCCESS;
}

twi_result_t twi_write_packet(
	uint8_t slave_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len
) {
	if ((len > 0U) && (data == (const uint8_t *)0)) {
		return TWI_ERROR_PARAM;
	}

	twi_result_t res = twi_start(slave_addr, TWI_WRITE);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	res = twi_write(reg_addr);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	for (uint8_t i = 0U; i < len; i++) {
		res = twi_write(data[i]);
		if (res != TWI_SUCCESS) {
			(void)twi_stop();
			return res;
		}
	}

	return twi_stop();
}

twi_result_t twi_read_packet(
	uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len
) {
	if ((len == 0U) || (data == (uint8_t *)0)) {
		return TWI_ERROR_PARAM;
	}

	/* Address target register via write */
	twi_result_t res = twi_start(slave_addr, TWI_WRITE);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	res = twi_write(reg_addr);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	/* Switch direction using Repeated START */
	res = twi_start(slave_addr, TWI_READ);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	/* Read len - 1 bytes with ACK */
	for (uint8_t i = 0U; i < (len - 1U); i++) {
		res = twi_read_ack(&data[i]);
		if (res != TWI_SUCCESS) {
			(void)twi_stop();
			return res;
		}
	}

	/* Read final byte with NACK to end transmission */
	res = twi_read_nack(&data[len - 1U]);
	if (res != TWI_SUCCESS) {
		(void)twi_stop();
		return res;
	}

	return twi_stop();
}
