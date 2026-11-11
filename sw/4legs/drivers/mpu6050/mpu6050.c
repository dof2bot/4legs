/**
 * @file mpu6050.c
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
 * @brief Implementation of MPU-6050 6-DOF IMU driver with low-pass filtering.
 */

#include "mpu6050.h"

#include "twi.h"

#include <math.h>
#include <util/delay.h>

enum {
	MPU6050_REG_CONFIG = 0x1AU,
	MPU6050_REG_GYRO_CONFIG = 0x1BU,
	MPU6050_REG_ACCEL_CONFIG = 0x1CU,
	MPU6050_REG_ACCEL_XOUT_H = 0x3BU,
	MPU6050_REG_PWR_MGMT_1 = 0x6BU,
	MPU6050_REG_WHO_AM_I = 0x75U,

	MPU6050_DATA_BURST_LEN = 14U,
	SHIFT_8 = 8U
};

static uint8_t s_i2c_addr = (uint8_t)MPU6050_I2C_ADDR_DEFAULT;
static bool s_ready = false;

static const imu_ops_t s_mpu6050_ops = {
    .read_data = mpu6050_read_data,
    .is_ready = mpu6050_is_ready
};

bool mpu6050_init(const mpu6050_config_t *config) {
	if (config != (const mpu6050_config_t *)0) {
		s_i2c_addr = config->i2c_addr;
	} else {
		s_i2c_addr = (uint8_t)MPU6050_I2C_ADDR_DEFAULT;
	}

	twi_init(TWI_FREQ_400KHZ);

	/* 1. Wake up MPU-6050: clear SLEEP bit in PWR_MGMT_1 */
	uint8_t pwr_data = 0x00U;
	uint8_t const pwr_reg = (uint8_t)MPU6050_REG_PWR_MGMT_1;
	if (twi_write_packet(s_i2c_addr, pwr_reg, &pwr_data, 1U) !=
	    TWI_SUCCESS) {
		s_ready = false;
		return false;
	}

	_delay_ms(10);

	/* 2. Configure DLPF (DLPF_CFG = 3 -> 44 Hz accel, 42 Hz gyro) */
	uint8_t dlpf_data = 0x03U;
	uint8_t const cfg_reg = (uint8_t)MPU6050_REG_CONFIG;
	(void)twi_write_packet(s_i2c_addr, cfg_reg, &dlpf_data, 1U);

	/* 3. Configure Gyro Full Scale (+/- 250 deg/s) */
	uint8_t gyro_cfg = 0x00U;
	uint8_t const gyro_reg = (uint8_t)MPU6050_REG_GYRO_CONFIG;
	(void)twi_write_packet(s_i2c_addr, gyro_reg, &gyro_cfg, 1U);

	/* 4. Configure Accel Full Scale (+/- 2g) */
	uint8_t accel_cfg = 0x00U;
	uint8_t const acc_reg = (uint8_t)MPU6050_REG_ACCEL_CONFIG;
	(void)twi_write_packet(s_i2c_addr, acc_reg, &accel_cfg, 1U);

	s_ready = true;
	return true;
}

bool mpu6050_read_data(void *context, imu_data_t *data) {
	(void)context;
	if ((data == (imu_data_t *)0) || (!s_ready)) {
		return false;
	}

	uint8_t burst[MPU6050_DATA_BURST_LEN];
	uint8_t const reg_xout = (uint8_t)MPU6050_REG_ACCEL_XOUT_H;
	uint8_t const burst_len = (uint8_t)MPU6050_DATA_BURST_LEN;
	if (twi_read_packet(s_i2c_addr, reg_xout, burst, burst_len) !=
	    TWI_SUCCESS) {
		return false;
	}

	uint16_t const u_ax =
		((uint16_t)burst[0] << SHIFT_8) | (uint16_t)burst[1];
	uint16_t const u_ay =
		((uint16_t)burst[2] << SHIFT_8) | (uint16_t)burst[3];
	uint16_t const u_az =
		((uint16_t)burst[4] << SHIFT_8) | (uint16_t)burst[5];
	uint16_t const u_gz =
		((uint16_t)burst[12] << SHIFT_8) | (uint16_t)burst[13];
	int16_t const raw_ax = (int16_t)u_ax;
	int16_t const raw_ay = (int16_t)u_ay;
	int16_t const raw_az = (int16_t)u_az;
	int16_t const raw_gz = (int16_t)u_gz;

	/* Scale accelerations to milli-g (+/- 2g -> 16384 LSB/g) */
	data->accel_x = (int16_t)(((int32_t)raw_ax * 1000L) / 16384L);
	data->accel_y = (int16_t)(((int32_t)raw_ay * 1000L) / 16384L);
	data->accel_z = (int16_t)(((int32_t)raw_az * 1000L) / 16384L);

	/* Scale gyro yaw rate to tenths of deg/s (131.0 LSB / deg/s) */
	data->gyro_z = (int16_t)(((int32_t)raw_gz * 10L) / 131L);

	/* Compute roll and pitch attitude angles in tenths of a degree */
	float const ax_f = (float)data->accel_x;
	float const ay_f = (float)data->accel_y;
	float const az_f = (float)data->accel_z;

	float const roll_rad = atan2f(ay_f, az_f);
	float const pitch_rad = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f));

	data->roll = (int16_t)(roll_rad * (1800.0f / 3.14159265f));
	data->pitch = (int16_t)(pitch_rad * (1800.0f / 3.14159265f));

	return true;
}

bool mpu6050_is_ready(void *context) {
	(void)context;
	return s_ready;
}

imu_driver_t mpu6050_get_driver(void) {
	imu_driver_t driver;
	driver.ops = &s_mpu6050_ops;
	driver.context = (void *)0;
	return driver;
}

