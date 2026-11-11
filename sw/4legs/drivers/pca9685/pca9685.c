/**
 * @file pca9685.c
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
 * @brief Implementation of PCA9685 16-channel 12-bit I2C PWM servo driver.
 */

#include "pca9685.h"

#include "twi.h"

#include <util/delay.h>

static const uint32_t PCA9685_PWM_RESOLUTION = 4096UL;
static const uint32_t PCA9685_PWM_PERIOD_US = 20000UL;

/**
 * @brief Physical hardware channel mapping on PCA9685 breakout board.
 *
 * Map:
 *   Leg 0 (FR): Coxa=CH2,  Femur=CH0,  Tibia=CH1
 *   Leg 1 (FL): Coxa=CH10, Femur=CH8,  Tibia=CH9
 *   Leg 2 (BR): Coxa=CH6,  Femur=CH4,  Tibia=CH5
 *   Leg 3 (BL): Coxa=CH14, Femur=CH12, Tibia=CH13
 */
static const uint8_t
    PCA9685_CHANNEL_MAP[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG] = {
	[LEG_FRONT_RIGHT] = {2U, 0U, 1U},
	[LEG_FRONT_LEFT] = {10U, 8U, 9U},
	[LEG_BACK_RIGHT] = {6U, 4U, 5U},
	[LEG_BACK_LEFT] = {14U, 12U, 13U}
};

static uint8_t s_i2c_addr = (uint8_t)PCA9685_I2C_ADDR_DEFAULT;
static bool s_use_staggering = true;
static bool s_attached = false;
static uint16_t s_stored_pulses[ROBOT_LEGS_COUNT][ROBOT_JOINTS_PER_LEG];

bool pca9685_init(const pca9685_config_t *config) {
	if (config != (const pca9685_config_t *)0) {
		s_i2c_addr = config->i2c_addr;
		s_use_staggering = config->use_staggering;
	} else {
		s_i2c_addr = (uint8_t)PCA9685_I2C_ADDR_DEFAULT;
		s_use_staggering = true;
	}

	twi_init(TWI_FREQ_400KHZ);

	/* Put PCA9685 into SLEEP mode to allow PRE_SCALE programming */
	uint8_t mode1_val = (uint8_t)PCA9685_MODE1_SLEEP;
	uint8_t const reg_m1 = (uint8_t)PCA9685_REG_MODE1;
	if (twi_write_packet(s_i2c_addr, reg_m1, &mode1_val, 1U) !=
	    TWI_SUCCESS) {
		return false;
	}

	/* Program 50 Hz PWM clock prescaler */
	uint8_t prescale_val = (uint8_t)PCA9685_PRESCALE_50HZ;
	uint8_t const reg_ps = (uint8_t)PCA9685_REG_PRE_SCALE;
	if (twi_write_packet(s_i2c_addr, reg_ps, &prescale_val, 1U) !=
	    TWI_SUCCESS) {
		return false;
	}

	/* Wake up PCA9685 and enable Auto-Increment addressing */
	mode1_val = (uint8_t)PCA9685_MODE1_AI;
	if (twi_write_packet(s_i2c_addr, reg_m1, &mode1_val, 1U) !=
	    TWI_SUCCESS) {
		return false;
	}

	/* Wait for internal oscillator stabilization */
	_delay_us((double)PCA9685_OSC_WAKEUP_DELAY_US);

	/* Set RESTART bit in MODE1 */
	mode1_val = (uint8_t)((uint8_t)PCA9685_MODE1_AI |
	                      (uint8_t)PCA9685_MODE1_RESTART);
	if (twi_write_packet(s_i2c_addr, reg_m1, &mode1_val, 1U) !=
	    TWI_SUCCESS) {
		return false;
	}

	/* Configure MODE2: Totem-pole outputs */
	uint8_t mode2_val = (uint8_t)PCA9685_MODE2_OUTDRV;
	uint8_t const reg_m2 = (uint8_t)PCA9685_REG_MODE2;
	if (twi_write_packet(s_i2c_addr, reg_m2, &mode2_val, 1U) !=
	    TWI_SUCCESS) {
		return false;
	}

	s_attached = true;

	/* Initialize all 12 servo channels to neutral position (1500us) */
	uint16_t const mid_p = (uint16_t)PCA9685_SAFE_MID_PULSE_US;
	for (uint8_t leg = 0U; leg < (uint8_t)ROBOT_LEGS_COUNT; leg++) {
		for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; j++) {
			s_stored_pulses[leg][j] = mid_p;
			if (!pca9685_set_pulse_us(leg, j, mid_p)) {
				return false;
			}
		}
	}

	return true;
}

bool pca9685_set_pulse_us(uint8_t leg, uint8_t joint, uint16_t pulse_us) {
	if ((leg >= (uint8_t)ROBOT_LEGS_COUNT) ||
	    (joint >= (uint8_t)ROBOT_JOINTS_PER_LEG)) {
		return false;
	}

	/* Clamp pulse width within safe boundaries */
	if (pulse_us < (uint16_t)PCA9685_SAFE_MIN_PULSE_US) {
		pulse_us = (uint16_t)PCA9685_SAFE_MIN_PULSE_US;
	} else if (pulse_us > (uint16_t)PCA9685_SAFE_MAX_PULSE_US) {
		pulse_us = (uint16_t)PCA9685_SAFE_MAX_PULSE_US;
	} else {
		/* In bounds */
	}

	s_stored_pulses[leg][joint] = pulse_us;

	if (!s_attached) {
		return true;
	}

	uint8_t const channel = PCA9685_CHANNEL_MAP[leg][joint];

	/* Convert pulse duration in us to 12-bit PWM ticks (0..4095) */
	uint32_t const num = ((uint32_t)pulse_us * PCA9685_PWM_RESOLUTION) +
	                     (PCA9685_PWM_PERIOD_US / 2UL);
	uint16_t const pulse_ticks = (uint16_t)(num / PCA9685_PWM_PERIOD_US);

	/* Calculate ON/OFF ticks with optional phase staggering */
	uint8_t const s_idx =
		(uint8_t)((leg * (uint8_t)ROBOT_JOINTS_PER_LEG) + joint);
	uint16_t const stagger =
		(uint16_t)(s_idx * (uint16_t)PCA9685_STAGGER_STEP_TICKS);
	uint16_t const on_tick = s_use_staggering ? stagger : 0U;
	uint16_t const off_tick = (uint16_t)(on_tick + pulse_ticks);

	/* Prepare 4-byte burst payload: ON_L, ON_H, OFF_L, OFF_H */
	uint8_t buf[PCA9685_BYTES_PER_CHANNEL];
	uint8_t const s8 = (uint8_t)PCA9685_BYTE_SHIFT_8;
	uint8_t const m_ff = (uint8_t)PCA9685_BYTE_MASK_FF;
	uint8_t const m_0f = (uint8_t)PCA9685_NIBBLE_MASK_0F;
	buf[0] = (uint8_t)(on_tick & m_ff);
	buf[1] = (uint8_t)((on_tick >> s8) & m_0f);
	buf[2] = (uint8_t)(off_tick & m_ff);
	buf[3] = (uint8_t)((off_tick >> s8) & m_0f);

	uint8_t const bpc = (uint8_t)PCA9685_BYTES_PER_CHANNEL;
	uint8_t const reg_start =
		(uint8_t)((uint8_t)PCA9685_REG_LED0_ON_L + (channel * bpc));

	return (twi_write_packet(s_i2c_addr, reg_start, buf, bpc) ==
	        TWI_SUCCESS);
}

bool pca9685_set_angle_tenths(uint8_t leg, uint8_t joint, int16_t tenths) {
	if (tenths < 0) {
		tenths = 0;
	} else if (tenths > 1800) {
		tenths = 1800;
	} else {
		/* In bounds */
	}

	/* Interpolation from tenths (0..1800) to pulse (500..2400 us) */
	uint16_t const p_min = (uint16_t)PCA9685_SAFE_MIN_PULSE_US;
	uint16_t const p_max = (uint16_t)PCA9685_SAFE_MAX_PULSE_US;
	uint32_t const span = (uint32_t)(p_max - p_min);
	uint16_t const delta = (uint16_t)(((uint32_t)tenths * span) / 1800UL);
	uint16_t const pulse_us = (uint16_t)(p_min + delta);

	return pca9685_set_pulse_us(leg, joint, pulse_us);
}

bool pca9685_set_angle_deg(uint8_t leg, uint8_t joint, float angle_deg) {
	int16_t const tenths = (int16_t)(angle_deg * 10.0f);
	return pca9685_set_angle_tenths(leg, joint, tenths);
}

bool pca9685_attach(void) {
	s_attached = true;
	bool success = true;

	for (uint8_t leg = 0U; leg < (uint8_t)ROBOT_LEGS_COUNT; leg++) {
		for (uint8_t j = 0U; j < (uint8_t)ROBOT_JOINTS_PER_LEG; j++) {
			uint16_t const p = s_stored_pulses[leg][j];
			if (!pca9685_set_pulse_us(leg, j, p)) {
				success = false;
			}
		}
	}

	return success;
}

bool pca9685_detach(void) {
	s_attached = false;
	uint8_t const bpc = (uint8_t)PCA9685_BYTES_PER_CHANNEL;
	const uint8_t buf[PCA9685_BYTES_PER_CHANNEL] = {
		0U, 0U, 0U, (uint8_t)PCA9685_LED_FULL_OFF_BIT
	};
	uint8_t const reg_all = (uint8_t)PCA9685_REG_ALL_LED_ON_L;
	return (twi_write_packet(s_i2c_addr, reg_all, buf, bpc) == TWI_SUCCESS);
}

bool pca9685_is_attached(void) { return s_attached; }

static bool adapter_set_pulse_us(
	void *context, uint8_t leg, uint8_t joint, uint16_t pulse_us
) {
	(void)context;
	return pca9685_set_pulse_us(leg, joint, pulse_us);
}

static bool adapter_set_angle_tenths(
	void *context, uint8_t leg, uint8_t joint, int16_t tenths
) {
	(void)context;
	return pca9685_set_angle_tenths(leg, joint, tenths);
}

static bool adapter_set_angle_deg(
	void *context, uint8_t leg, uint8_t joint, float angle_deg
) {
	(void)context;
	return pca9685_set_angle_deg(leg, joint, angle_deg);
}

static bool adapter_attach(void *context) {
	(void)context;
	return pca9685_attach();
}

static bool adapter_detach(void *context) {
	(void)context;
	return pca9685_detach();
}

static bool adapter_is_attached(void *context) {
	(void)context;
	return pca9685_is_attached();
}

static const servo_ops_t s_pca9685_ops = {
    adapter_set_pulse_us,
    adapter_set_angle_tenths,
    adapter_set_angle_deg,
    adapter_attach,
    adapter_detach,
    adapter_is_attached
};

servo_driver_t pca9685_get_driver(void) {
	servo_driver_t driver;
	driver.ops = &s_pca9685_ops;
	driver.context = (void *)0;

	return driver;
}
