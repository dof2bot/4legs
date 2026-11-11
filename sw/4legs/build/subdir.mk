# subdir.mk
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

include tools.mk
include cflags.mk

C_SRCS += \
	../main.c \
	../core/ring_buffer/ring_buffer.c \
	../core/mem_pool/mem_pool.c \
	../core/crc8/crc8.c \
	../core/motion/motion_planner.c \
	../core/motion/gait_controller.c \
	../core/command_parser/frame_parser.c \
	../core/command_parser/command_dispatcher.c \
	../drivers/uart/uart.c \
	../drivers/timer/timer.c \
	../drivers/twi/twi.c \
	../drivers/pca9685/pca9685.c \
	../drivers/mpu6050/mpu6050.c

OBJS += \
	./main.o \
	./ring_buffer.o \
	./mem_pool.o \
	./crc8.o \
	./motion_planner.o \
	./gait_controller.o \
	./frame_parser.o \
	./command_dispatcher.o \
	./uart.o \
	./timer.o \
	./twi.o \
	./pca9685.o \
	./mpu6050.o

C_DEPS += \
	./main.d \
	./ring_buffer.d \
	./mem_pool.d \
	./crc8.d \
	./motion_planner.d \
	./gait_controller.d \
	./frame_parser.d \
	./command_dispatcher.d \
	./uart.d \
	./timer.d \
	./twi.d \
	./pca9685.d \
	./mpu6050.d

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../core/ring_buffer/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../core/mem_pool/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../core/crc8/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../core/motion/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../core/command_parser/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../drivers/uart/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../drivers/timer/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../drivers/twi/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../drivers/pca9685/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../drivers/mpu6050/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	$(CC) $(CCFLAGS) "$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
