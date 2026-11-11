/**
 * @file pca9685.h
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
 * @brief PCA9685 16-channel 12-bit I2C PWM servo controller driver.
 *
 * Implements the abstract iservo.h (servo_driver_t) interface over hardware TWI/I2C.
 * Controls 12 robot joint servos (4 legs x 3 joints) with hardware 50Hz PWM, 12-bit
 * resolution, and phase-staggered turn-on to eliminate power bus current surges.
 */

#pragma once

#include "iservo.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default 7-bit I2C address for PCA9685 breakout board.
 */
typedef enum {
	PCA9685_I2C_ADDR_DEFAULT = 0x40U /**< Default I2C address (A0..A5 pulled to GND). */
} pca9685_i2c_address_t;

/**
 * @brief PCA9685 internal register addresses.
 */
typedef enum {
	PCA9685_REG_MODE1 = 0x00U,         /**< Mode register 1. */
	PCA9685_REG_MODE2 = 0x01U,         /**< Mode register 2. */
	PCA9685_REG_SUBADR1 = 0x02U,       /**< I2C-bus subaddress 1. */
	PCA9685_REG_SUBADR2 = 0x03U,       /**< I2C-bus subaddress 2. */
	PCA9685_REG_SUBADR3 = 0x04U,       /**< I2C-bus subaddress 3. */
	PCA9685_REG_ALLCALLADR = 0x05U,    /**< LED All Call I2C-bus address. */
	PCA9685_REG_LED0_ON_L = 0x06U,     /**< Channel 0 ON low byte. */
	PCA9685_REG_LED0_ON_H = 0x07U,     /**< Channel 0 ON high byte. */
	PCA9685_REG_LED0_OFF_L = 0x08U,    /**< Channel 0 OFF low byte. */
	PCA9685_REG_LED0_OFF_H = 0x09U,    /**< Channel 0 OFF high byte. */
	PCA9685_REG_ALL_LED_ON_L = 0xFAU,  /**< All channels ON low byte. */
	PCA9685_REG_ALL_LED_ON_H = 0xFBU,  /**< All channels ON high byte. */
	PCA9685_REG_ALL_LED_OFF_L = 0xFCU, /**< All channels OFF low byte. */
	PCA9685_REG_ALL_LED_OFF_H = 0xFDU, /**< All channels OFF high byte. */
	PCA9685_REG_PRE_SCALE = 0xFEU,     /**< Prescaler for PWM output frequency. */
	PCA9685_REG_TEST_MODE = 0xFFU      /**< Test mode register. */
} pca9685_registers_t;

/**
 * @brief PCA9685 MODE1 register bit definitions.
 */
typedef enum {
	PCA9685_MODE1_ALLCALL = 0x01U, /**< Respond to LED All Call address. */
	PCA9685_MODE1_SUB3 = 0x02U,    /**< Respond to subaddress 3. */
	PCA9685_MODE1_SUB2 = 0x04U,    /**< Respond to subaddress 2. */
	PCA9685_MODE1_SUB1 = 0x08U,    /**< Respond to subaddress 1. */
	PCA9685_MODE1_SLEEP = 0x10U,   /**< Low power mode / oscillator off. */
	PCA9685_MODE1_AI = 0x20U,      /**< Auto-Increment register address enable. */
	PCA9685_MODE1_EXTCLK = 0x40U,  /**< Use external clock source. */
	PCA9685_MODE1_RESTART = 0x80U  /**< Restart enabled. */
} pca9685_mode1_bits_t;

/**
 * @brief PCA9685 MODE2 register bit definitions.
 */
typedef enum {
	PCA9685_MODE2_OUTNE_0 = 0x01U, /**< Output driver configuration bit 0. */
	PCA9685_MODE2_OUTNE_1 = 0x02U, /**< Output driver configuration bit 1. */
	PCA9685_MODE2_OUTDRV = 0x04U,  /**< Totem-pole (push-pull) outputs (for RC servos). */
	PCA9685_MODE2_OCH = 0x08U,     /**< Outputs change on I2C ACK instead of STOP. */
	PCA9685_MODE2_INVRT = 0x10U    /**< Output logic state inverted. */
} pca9685_mode2_bits_t;

enum {
	PCA9685_PRESCALE_50HZ = 121U,     /**< round(25MHz / (4096 * 50Hz)) - 1 = 121 (0x79). */
	PCA9685_BYTES_PER_CHANNEL = 4U,   /**< 4 registers per channel (ON_L, ON_H, OFF_L, OFF_H). */
	PCA9685_LED_FULL_OFF_BIT = 0x10U, /**< Bit 4 of LEDn_OFF_H forces channel full OFF. */

	PCA9685_SAFE_MIN_PULSE_US = 500U,  /**< Minimum allowable pulse width (microseconds). */
	PCA9685_SAFE_MID_PULSE_US = 1500U, /**< Neutral center pulse width (microseconds). */
	PCA9685_SAFE_MAX_PULSE_US = 2400U, /**< Maximum allowable pulse width (microseconds). */

	PCA9685_STAGGER_STEP_TICKS = 256U, /**< 256 ticks (~1.25ms) phase offset per channel. */
	PCA9685_BYTE_SHIFT_8 = 8U,         /**< 8-bit shift constant. */
	PCA9685_BYTE_MASK_FF = 0xFFU,      /**< 8-bit mask constant. */
	PCA9685_NIBBLE_MASK_0F = 0x0FU,    /**< 4-bit nibble mask for 12-bit register upper byte. */
	PCA9685_OSC_WAKEUP_DELAY_US = 500U /**< Oscillator stabilization delay (microseconds). */
};

/**
 * @brief PCA9685 driver configuration parameters.
 */
typedef struct {
	uint8_t i2c_addr;    /**< 7-bit I2C address (default PCA9685_I2C_ADDR_DEFAULT = 0x40). */
	bool use_staggering; /**< Enable phase staggering to eliminate simultaneous current peaks. */
} pca9685_config_t;

/**
 * @brief Initializes PCA9685 PWM controller and hardware TWI interface.
 *
 * Configures 50 Hz PWM refresh rate, totem-pole outputs, auto-increment addressing,
 * sets all 12 servo channels to neutral position (1500us), and attaches driver.
 *
 * @param config Optional pointer to custom configuration, or NULL for defaults.
 * @return True on successful initialization, false on I2C communication error.
 */
bool pca9685_init(const pca9685_config_t *config);

/**
 * @brief Sets PWM pulse width for a specified leg and joint in microseconds.
 *
 * @param leg Leg index (LEG_FRONT_RIGHT .. LEG_BACK_LEFT).
 * @param joint Joint index (JOINT_COXA .. JOINT_TIBIA).
 * @param pulse_us Pulse width in microseconds (clamped to safe limits).
 * @return True if command successfully transmitted, false on invalid parameter or I2C error.
 */
bool pca9685_set_pulse_us(uint8_t leg, uint8_t joint, uint16_t pulse_us);

/**
 * @brief Sets shaft angle for a specified leg and joint in tenths of a degree (0..1800).
 *
 * Uses integer scaling to convert angle to pulse width without float division.
 *
 * @param leg Leg index (LEG_FRONT_RIGHT .. LEG_BACK_LEFT).
 * @param joint Joint index (JOINT_COXA .. JOINT_TIBIA).
 * @param tenths Joint angle in tenths of a degree (0 .. 1800).
 * @return True if successfully updated, false on error.
 */
bool pca9685_set_angle_tenths(uint8_t leg, uint8_t joint, int16_t tenths);

/**
 * @brief Sets shaft angle for a specified leg and joint in degrees (0..180).
 *
 * Linearly converts angle to pulse width and updates PCA9685 PWM channel.
 *
 * @param leg Leg index (LEG_FRONT_RIGHT .. LEG_BACK_LEFT).
 * @param joint Joint index (JOINT_COXA .. JOINT_TIBIA).
 * @param angle_deg Joint angle in degrees (0.0 .. 180.0).
 * @return True if successfully updated, false on error.
 */
bool pca9685_set_angle_deg(uint8_t leg, uint8_t joint, float angle_deg);

/**
 * @brief Re-attaches and re-enables all servo PWM outputs.
 *
 * Restores the stored pulse widths to all 12 servo channels.
 *
 * @return True if successfully attached, false on error.
 */
bool pca9685_attach(void);

/**
 * @brief Detaches and disables all servo PWM outputs (cuts drive signal).
 *
 * Sets the full-OFF bit on all channels to put servos into idle/unpowered state.
 *
 * @return True if successfully detached, false on error.
 */
bool pca9685_detach(void);

/**
 * @brief Checks if PCA9685 servo driver is currently attached.
 *
 * @return True if attached, false if detached.
 */
bool pca9685_is_attached(void);

/**
 * @brief Factory function producing polymorphic iservo.h driver instance.
 *
 * @return Configured servo_driver_t struct bound to PCA9685 operations.
 */
servo_driver_t pca9685_get_driver(void);

#ifdef __cplusplus
}
#endif
