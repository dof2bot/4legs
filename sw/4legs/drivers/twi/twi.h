/**
 * @file twi.h
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
 * @brief Hardware Two-Wire Interface (TWI/I2C) master driver for ATmega328P.
 *
 * Provides bounded-timeout, deterministic TWI master transmission and reception
 * complying with MISRA-C rules for communication with external I2C peripherals.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TWI communication result and error status codes.
 */
typedef enum {
	TWI_SUCCESS = 0U,         /**< Operation completed successfully with ACK. */
	TWI_ERROR_START = 1U,     /**< Failed to generate bus START or Repeated START condition. */
	TWI_ERROR_SLA_NACK = 2U,  /**< Slave device returned NACK on address transmission. */
	TWI_ERROR_DATA_NACK = 3U, /**< Slave device returned NACK on data byte transmission. */
	TWI_ERROR_TIMEOUT = 4U,   /**< Bus operation timed out (bounded wait violation). */
	TWI_ERROR_PARAM = 5U      /**< Invalid null buffer or length parameter. */
} twi_result_t;

/**
 * @brief Supported TWI SCL clock frequency bitrates at 16 MHz F_CPU (Prescaler = 1).
 */
typedef enum {
	TWI_FREQ_100KHZ = 72U, /**< Standard Mode 100 kHz (TWBR = 72). */
	TWI_FREQ_400KHZ = 12U  /**< Fast Mode 400 kHz (TWBR = 12). */
} twi_bitrate_t;

/**
 * @brief Direction indicator for SLA+W or SLA+R bus addressing.
 */
typedef enum {
	TWI_WRITE = 0U, /**< Master transmit / write operation (R/W bit = 0). */
	TWI_READ = 1U   /**< Master receive / read operation (R/W bit = 1). */
} twi_op_mode_t;

enum {
	TWI_TIMEOUT_CYCLES = 10000U, /**< Bounded loop iteration limit preventing infinite stalls. */
	TWI_PRESCALER_1 = 0U,        /**< Prescaler 1 value for TWPS bits in TWSR. */
	TWI_STATUS_MASK = 0xF8U,     /**< Mask for status code bits 7..3 in TWSR. */
	TWI_SLA_READ_BIT = 1U        /**< LSB bit 0 indicates read in SLA byte. */
};

/**
 * @brief Initializes ATmega328P hardware TWI peripheral in master mode.
 *
 * Configures internal pull-ups on SDA (PC4) and SCL (PC5), sets bit rate register
 * for requested bus clock, and enables the TWI hardware engine.
 *
 * @param bitrate Desired bus frequency configuration (100 kHz or 400 kHz).
 */
void twi_init(twi_bitrate_t bitrate);

/**
 * @brief Generates START condition and transmits slave address with R/W direction.
 *
 * @param slave_addr 7-bit I2C slave address (e.g. 0x40).
 * @param mode Direction of transfer (TWI_WRITE or TWI_READ).
 * @return TWI_SUCCESS on SLA ACK, or error code on failure/timeout.
 */
twi_result_t twi_start(uint8_t slave_addr, twi_op_mode_t mode);

/**
 * @brief Transmits a single data byte to slave device and verifies ACK.
 *
 * @param data Data byte to transmit.
 * @return TWI_SUCCESS on ACK, or error code on NACK/timeout.
 */
twi_result_t twi_write(uint8_t data);

/**
 * @brief Reads a single data byte from slave device, returning ACK to continue.
 *
 * @param data Pointer to destination byte.
 * @return TWI_SUCCESS on successful reception, or error code.
 */
twi_result_t twi_read_ack(uint8_t *data);

/**
 * @brief Reads a single final data byte from slave device, returning NACK to terminate.
 *
 * @param data Pointer to destination byte.
 * @return TWI_SUCCESS on successful reception, or error code.
 */
twi_result_t twi_read_nack(uint8_t *data);

/**
 * @brief Generates bus STOP condition and waits until hardware clears STOP flag.
 *
 * @return TWI_SUCCESS on successful bus release, or TWI_ERROR_TIMEOUT.
 */
twi_result_t twi_stop(void);

/**
 * @brief High-level helper to write a block of bytes to a slave register.
 *
 * Performs complete sequence: START -> SLA+W -> Register -> Data[0..len-1] -> STOP.
 *
 * @param slave_addr 7-bit I2C slave address.
 * @param reg_addr Target register address on slave device.
 * @param data Pointer to buffer of bytes to transmit.
 * @param len Number of bytes to transmit from buffer.
 * @return TWI_SUCCESS on full transmission, or specific error code.
 */
twi_result_t twi_write_packet(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len);

/**
 * @brief High-level helper to read a block of bytes from a slave register.
 *
 * Performs complete sequence: START -> SLA+W -> Register -> REP_START -> SLA+R -> Data -> STOP.
 *
 * @param slave_addr 7-bit I2C slave address.
 * @param reg_addr Target register address on slave device.
 * @param data Pointer to destination buffer.
 * @param len Number of bytes to read into buffer.
 * @return TWI_SUCCESS on full reception, or specific error code.
 */
twi_result_t twi_read_packet(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif
