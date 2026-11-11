4legs
=====

.. toctree::
   :hidden:

   self

**4legs** is a modular, high-reliability embedded firmware and hardware control framework for a 12-DOF (degree-of-freedom) quadruped / spider robot based on the **Microchip AVR ATmega328P** microcontroller.

The firmware is developed in **100% bare-metal C** adhering to **Clean Architecture**, **SOLID principles**, and **MISRA-C:2012 / 2023** deterministic embedded standards: zero dynamic memory allocation, bounded execution loops, lock-free ring buffers, and CRC-8 integrity verification on all serial communication.

|GitHub issues| |Documentation Status| |GitHub contributors| |License: GPL v3|

.. |GitHub issues| image:: https://img.shields.io/github/issues/dof2bot/4legs.svg
   :target: https://github.com/dof2bot/4legs/issues

.. |GitHub contributors| image:: https://img.shields.io/github/contributors/dof2bot/4legs.svg
   :target: https://github.com/dof2bot/4legs/graphs/contributors

.. |Documentation Status| image:: https://readthedocs.org/projects/4legs/badge/?version=latest
   :target: https://4legs.readthedocs.io/projects/4legs/en/latest/?badge=latest

.. |License: GPL v3| image:: https://img.shields.io/badge/License-GPLv3-blue.svg
   :target: https://www.gnu.org/licenses/gpl-3.0

Overview & Robotics Design
--------------------------

Quadruped locomotion requires coordinated multi-axis kinematics, smooth trajectory interpolation, and real-time attitude stabilization. **4legs** implements the low-level embedded real-time firmware that drives the robot actuators, reads inertial sensors, and communicates with higher-level planning software over an asynchronous binary protocol.

.. code-block:: text

          [Leg 1: Front-Left]                [Leg 0: Front-Right]
                 \                                  /
                  +--------------------------------+
                  |        4legs Quadruped         |
                  |       ATmega328P + MPU         |
                  |          PCA9685 PWM           |
                  +--------------------------------+
                 /                                  \
          [Leg 2: Rear-Left]                 [Leg 3: Rear-Right]

Each leg comprises 3 degrees of freedom:

1. **Coxa (Joint 0)**: Horizontal hip yaw/abduction (:math:`\pm 45^\circ`).
2. **Femur (Joint 1)**: Vertical thigh pitch (:math:`-60^\circ` to :math:`+60^\circ`).
3. **Tibia (Joint 2)**: Vertical knee pitch (:math:`-90^\circ` to :math:`+30^\circ`).

Key Features
------------

* **Bare-metal AVR Execution**: Optimized for ATmega328P clocked at 16.0 MHz (5V logic).
* **Clean Architecture & Ports/Adapters**: Hardware registers and low-level drivers (``uart``, ``twi``, ``pca9685``, ``mpu6050``) are fully decoupled from core motion planning and command dispatching via polymorphic interfaces.
* **MISRA-C Determinism**: Zero dynamic memory allocation (``malloc``/``free`` strictly forbidden). Static memory pool (``mem_pool``) for motion keyframes with :math:`O(1)` allocation/deallocation.
* **16-Channel 12-bit Servo Control**: Hardware I2C / TWI interface to **PCA9685** PWM controller delivering jitter-free 50 Hz servo pulses with microsecond resolution.
* **6-DOF IMU Telemetry & Attitude Estimation**: Integrated **MPU-6050** driver running a digital low-pass filter (DLPF 44 Hz) and onboard attitude estimation calculating pitch and roll tilt angles in integer tenths of a degree (:math:`0.1^\circ`).
* **Robust Binary Protocol**: Packet framing with ``0xAA`` synchronization, sequence numbering, payload lengths, and **CRC-8** (polynomial ``0x07``, ATM/SMBus) integrity checking.
* **Dual Safety Watchdogs**: Microcontroller hardware Watchdog Timer (WDT) and software motion timeout supervision to guarantee robot safety in case of connection loss.

Hardware Specifications & Pinout
--------------------------------

Bill of Materials
~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 20 40 40
   :header-rows: 1

   * - Component
     - Description
     - Interface / Notes
   * - **MCU**
     - Microchip ATmega328P (16 MHz crystal, 5V)
     - Arduino Nano / Pro Mini or custom PCB
   * - **PWM Driver**
     - NXP PCA9685 16-Channel 12-bit PWM Controller
     - I2C (address ``0x40``), 50 Hz servo refresh
   * - **IMU Sensor**
     - InvenSense MPU-6050 (6-axis gyro + accelerometer)
     - I2C (address ``0x68``), onboard DLPF filter
   * - **Servos**
     - 12x MG90S (Metal Gear) or SG90 Micro Servos
     - 3 servos per leg :math:`\times` 4 legs
   * - **Power Supply**
     - Dual-rail: 5V logic & dedicated 5V/6V 4A+ BEC
     - High current rail isolated for servo inductive loads
   * - **Communication**
     - Hardware UART / USB-to-UART bridge (CH340/FT232)
     - 57600 baud, 8N1

Pinout Allocation
~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 25 50
   :header-rows: 1

   * - ATmega328P Pin
     - Port / Peripheral
     - Function / Connection
   * - **Pin 0**
     - ``PD0`` (RXD)
     - UART Serial Receive (57600 baud)
   * - **Pin 1**
     - ``PD1`` (TXD)
     - UART Serial Transmit (57600 baud)
   * - **Pin A4**
     - ``PC4`` (SDA)
     - Hardware I2C / TWI Data line (PCA9685 + MPU-6050)
   * - **Pin A5**
     - ``PC5`` (SCL)
     - Hardware I2C / TWI Clock line (PCA9685 + MPU-6050)
   * - **Timer 0 / 2**
     - Internal 8-bit CTC
     - 1.0 ms system tick (``timer_millis()``) & 50 Hz motion clock

PCA9685 Servo Mapping
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   Leg 0 (Front Right): CH 0 (Coxa),  CH 1 (Femur),  CH 2 (Tibia)
   Leg 1 (Front Left):  CH 4 (Coxa),  CH 5 (Femur),  CH 6 (Tibia)
   Leg 2 (Rear Left):   CH 8 (Coxa),  CH 9 (Femur),  CH 10 (Tibia)
   Leg 3 (Rear Right):  CH 12 (Coxa), CH 13 (Femur), CH 14 (Tibia)

Firmware Architecture
---------------------

Dependencies strictly flow from the outside inwards:
``drivers/`` :math:`\longrightarrow` ``core/interfaces/`` :math:`\longleftarrow` ``core/domain``.

.. code-block:: text

   +-------------------------------------------------------------------+
   |                     MAIN ENTRYPOINT & SUPERLOOP                   |
   |           sw/4legs/main.c (Composition & Event Dispatcher)        |
   +-------------------------------------------------------------------+
                                     |
           +-------------------------+-------------------------+
           |                                                   |
           v                                                   v
   +-------------------------------+   +-------------------------------+
   |     APPLICATION SERVICES      |   |       CORE DOMAIN LOGIC       |
   | - command_dispatcher          |   | - motion_types                |
   | - gait_controller (FSM)       |   | - mem_pool (O(1) static pool) |
   | - motion_planner (queue)      |   | - crc8 (tableless polynomial) |
   | - frame_parser (stream FSM)   |   |                               |
   +-------------------------------+   +-------------------------------+
           |                                                   |
           +-------------------------+-------------------------+
                                     |
                                     v
   +-------------------------------------------------------------------+
   |                     ABSTRACT PORTS (interfaces/)                  |
   | - icomm.h         : Serial stream communication port              |
   | - iservo.h        : Servo pulse generation port                   |
   | - iimu.h          : 6-DOF IMU telemetry driver contract           |
   | - protocol_types.h: Binary packet format and ID constants         |
   +-------------------------------------------------------------------+
                                     ^
                                     | (Structural Implementation)
   +-------------------------------------------------------------------+
   |                        HARDWARE DRIVERS (drivers/)                |
   | - uart    : ISR-driven UART with lock-free SPSC ring buffers      |
   | - twi     : Non-blocking timeout-protected hardware I2C master    |
   | - pca9685 : 16-channel 12-bit PWM servo expansion driver         |
   | - mpu6050 : 6-axis accelerometer/gyroscope attitude estimation   |
   | - timer   : 1 ms tick timebase and motion interpolation clock     |
   +-------------------------------------------------------------------+

Binary Wire Protocol Specification
----------------------------------

Wire Frame Anatomy
~~~~~~~~~~~~~~~~~~

.. code-block:: text

   +--------+--------+--------+----------------------------+--------+
   |  SYNC  | MSG_ID | LENGTH |          PAYLOAD           | CRC-8  |
   | (0xAA) | (1 B)  | (1 B)  |         (0..16 B)          | (1 B)  |
   +--------+--------+--------+----------------------------+--------+
            |<---------------- CRC-8 Scope --------------->|

* **SYNC**: Constant start delimiter ``0xAA`` (170 dec).
* **MSG_ID**: 1-byte command or response identifier.
* **LENGTH**: 1-byte payload size (:math:`0 \le N \le 16`).
* **PAYLOAD**: Variable binary data payload.
* **CRC-8**: 1-byte checksum computed over ``[MSG_ID, LENGTH, PAYLOAD...]`` using polynomial :math:`x^8 + x^2 + x^1 + x^0` (``0x07``).

Command Set (Host to Robot)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 15 25 35 25
   :header-rows: 1

   * - Command ID
     - Identifier
     - Payload Format
     - Description
   * - ``0x01``
     - ``CMD_PING``
     - Empty
     - Heartbeat request, expects ``RESP_PONG``.
   * - ``0x02``
     - ``CMD_STOP``
     - Empty
     - Emergency stop, halts ongoing motion.
   * - ``0x10``
     - ``CMD_MOVE_LEG``
     - ``leg_idx`` (u8), ``coxa`` (i16), ``femur`` (i16), ``tibia`` (i16), ``duration`` (u16)
     - Move single leg joints to target angles (:math:`0.1^\circ`).
   * - ``0x11``
     - ``CMD_SET_ALL_LEGS``
     - 12 :math:`\times` angles (i16), ``duration`` (u16)
     - Coordinate target pose for all 4 legs simultaneously.
   * - ``0x20``
     - ``CMD_SET_GAIT``
     - ``gait_id`` (u8): ``0x00`` Stand, ``0x01`` Sit, ``0x02`` Walk, ``0x03`` Trot, ``0x04`` Wave
     - Switch onboard autonomous gait generator.
   * - ``0x21``
     - ``CMD_SET_SPEED``
     - ``multiplier`` (u8, 1..10)
     - Adjust motion execution speed factor.
   * - ``0x32``
     - ``CMD_GET_IMU``
     - Empty
     - Request real-time 6-DOF IMU attitude and telemetry.

Response Set (Robot to Host)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 15 25 35 25
   :header-rows: 1

   * - Response ID
     - Identifier
     - Payload Format
     - Description
   * - ``0x80``
     - ``RESP_PONG``
     - Empty
     - Heartbeat reply confirming online state.
   * - ``0x90``
     - ``RESP_ACK``
     - Empty
     - Command parsed, verified, and enqueued.
   * - ``0x91``
     - ``RESP_IMU``
     - 12 bytes big-endian: ``roll`` (i16), ``pitch`` (i16), ``ax`` (i16), ``ay`` (i16), ``az`` (i16), ``temp`` (i16)
     - 6-DOF attitude and accelerometer telemetry.
   * - ``0xFF``
     - ``RESP_NACK``
     - ``error_code`` (u8)
     - Checksum error, buffer overflow, or invalid parameter.

Prerequisites & Toolchain
-------------------------

On Debian / Ubuntu systems:

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y gcc-avr binutils-avr avr-libc avrdude make python3 python3-pip

Compilation & Flashing
----------------------

Building the Firmware
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd sw/4legs/build
   make clean
   make

Artifacts generated:
* ``4legs.hex``: Intel Hex file for flashing to MCU program memory.
* ``4legs.elf``: Executable and Linkable Format binary with debugging symbols.
* ``4legs.map``: Memory layout map showing exact flash and SRAM usage.
* ``4legs.lss``: Disassembly listing for timing and instruction verification.

Flashing to Hardware
~~~~~~~~~~~~~~~~~~~~

Using Arduino bootloader over USB serial:

.. code-block:: bash

   avrdude -v -p m328p -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:4legs.hex:i

Using USBasp In-System Programmer:

.. code-block:: bash

   avrdude -v -p m328p -c usbasp -U flash:w:4legs.hex:i

Host Companion Integration (tetrapoder)
---------------------------------------

The companion Python toolkit **tetrapoder** provides inverse kinematics, gait planning, trajectory compilation, telemetry decoding, and closed-loop balance stabilization.

Live Serial Execution
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Test serial heartbeat:
   tetrapoder --action ping --port /dev/ttyUSB0 --baud 57600

   # Execute walk gait directly on robot:
   tetrapoder --action run --gait walk --port /dev/ttyUSB0

   # Command robot to sit:
   tetrapoder --action run --gait sit --port /dev/ttyUSB0

Closed-Loop Balance API Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from tetrapoder.core.service.telemetry.decoder import TelemetryDecoder
   from tetrapoder.core.service.stabilizer.compensator import PostureCompensator
   from tetrapoder.infrastructure.transport.serial import SerialTransport

   transport = SerialTransport(port='/dev/ttyUSB0', baudrate=57600)
   transport.open()

   # Request IMU attitude
   transport.write(b'\xAA\x32\x00\x3A')
   response = transport.read(16)
   telemetry = TelemetryDecoder().decode(response[3:15])

   # Compensate leg height according to chassis tilt
   compensator = PostureCompensator(chassis_length=120.0, chassis_width=90.0)
   for leg in range(4):
       offset = compensator.compute_offset(leg_index=leg, roll_deg=telemetry.roll, pitch_deg=telemetry.pitch)
       print(f'Leg {leg}: vertical adjustment Δz = {offset.z:.2f} mm')

Copyright and Licence
---------------------

|License: GPL v3|

Copyright (C) 2020 - 2026 by Vladimir Roncevic <elektron.ronca@gmail.com>

**4legs** is free software: you can redistribute it and/or modify it under the terms of the **GNU General Public License** as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
