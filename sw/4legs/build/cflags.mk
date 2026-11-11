# cflags.mk
# Copyright (C) 2021 - 2026 Vladimir Roncevic <elektron.ronca@gmail.com>
#
# 4legs is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# 4legs is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program_name. If not, see <http://www.gnu.org/licenses/>.

INCLUDES := \
	-I.. \
	-I../core/interfaces \
	-I../core/mem_pool \
	-I../core/ring_buffer \
	-I../core/crc8 \
	-I../core/motion \
	-I../core/command_parser \
	-I../drivers/uart \
	-I../drivers/timer \
	-I../drivers/twi \
	-I../drivers/pca9685 \
	-I../drivers/mpu6050

LIBS := -lm

CFLAGS := \
	-Wl,-Map,4legs.map \
	-mmcu=atmega328p

CCFLAGS := \
	$(INCLUDES) \
	-Wall \
	-g2 \
	-O2 \
	-fpack-struct \
	-fshort-enums \
	-ffunction-sections \
	-fdata-sections \
	-std=gnu99 \
	-funsigned-char \
	-funsigned-bitfields \
	-mmcu=atmega328p \
	-DF_CPU=16000000UL \
	-MMD \
	-MP \
	-MF
