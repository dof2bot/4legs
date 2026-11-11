# 4legs Binary Wire Protocol & Firmware Architecture Specification

## 1. Architectural Overview

The **4legs** firmware is implemented in bare-metal C for the **Microchip AVR ATmega328P** microcontroller (16 MHz clock, 5V supply) driving a 4-legged (quadruped/crab) robot equipped with 12 SG90/MG90 servo motors (3 degrees of freedom per leg: *Coxa*, *Femur*, *Tibia*).

The codebase strictly adheres to:
1. **Clean Architecture & SOLID Principles**: Inward dependency flow (`drivers/` $\to$ `core/interfaces/` $\leftarrow$ `core/domain`). Higher-level business and motion planning logic depends exclusively on abstract interfaces (`icomm_t`, `iservo_driver_t`), with zero direct hardware register coupling.
2. **MISRA-C:2012 / 2023 Compliance**:
   - Zero dynamic memory allocation (`malloc`, `free` forbidden).
   - All memory statically allocated in `.bss`/`.data`.
   - Deterministic $O(1)$ memory pool (`mem_pool_t`) for motion keyframes.
   - SPSC lock-free ring buffers (`ring_buffer_t`) for UART RX and TX.
   - Fixed-width integer types (`<stdint.h>`) throughout.
   - Guarded input ranges, bounded loops, and NaN prevention on trigonometric functions.
   - Dual-watchdog safety (Hardware AVR WDT + software timeouts).

```
+-----------------------------------------------------------------+
|                   APPLICATION / COMPOSITION                     |
|            main.c (Dependency Injection & Superloop)            |
+-----------------------------------------------------------------+
                                |
        +-----------------------+-----------------------+
        |                                               |
        v                                               v
+-------------------------------+       +-------------------------------+
|     APPLICATION SERVICES      |       |      CORE DOMAIN LOGIC        |
| - command_dispatcher          |       | - motion_types                |
| - gait_controller (FSM)       |       | - mem_pool (O(1) allocator)   |
| - motion_planner (joint seq)  |       | - crc8 (ATM/SMBus)            |
| - frame_parser (FSM)          |       |                               |
+-------------------------------+       +-------------------------------+
        |                                               |
        +-----------------------+-----------------------+
                                |
                                v
+---------------------------------------------------------------+
|                   ABSTRACT PORTS (core/interfaces)            |
| - icomm.h       : Stream I/O (read, write, available)         |
| - iservo.h      : Servo controller (set_pulse, set_angle)     |
| - protocol_types.h: Binary wire framing constants             |
+---------------------------------------------------------------+
                                ^
                                | (Implements ports)
+---------------------------------------------------------------+
|                 DRIVERS / HAL (drivers/)                      |
| - uart       : Asynchronous ISR UART with RX/TX ring buffers  |
| - timer      : 1 ms system tick via 8-bit Timer2 CTC          |
| - twi        : Bounded-timeout hardware I2C master driver     |
| - pca9685    : 16-channel 12-bit 50Hz I2C PWM servo driver    |
+---------------------------------------------------------------+
```

---

## 2. Hardware Pinout & Timer Allocation

### 2.1 Servo Mapping (ATmega328P)
* **Leg 0 (Front Right)**:
  * Joint 0 (Coxa): Pin 2 (`PD2`)
  * Joint 1 (Femur): Pin 3 (`PD3`)
  * Joint 2 (Tibia): Pin 4 (`PD4`)
* **Leg 1 (Front Left)**:
  * Joint 0 (Coxa): Pin 5 (`PD5`)
  * Joint 1 (Femur): Pin 6 (`PD6`)
  * Joint 2 (Tibia): Pin 7 (`PD7`)
* **Leg 2 (Rear Left)**:
  * Joint 0 (Coxa): Pin 8 (`PB0`)
  * Joint 1 (Femur): Pin 9 (`PB1`)
  * Joint 2 (Tibia): Pin 10 (`PB2`)
* **Leg 3 (Rear Right)**:
  * Joint 0 (Coxa): Pin 11 (`PB3`)
  * Joint 1 (Femur): Pin 12 (`PB4`)
  * Joint 2 (Tibia): Pin 13 (`PB5`)

### 2.2 Peripherals & Timers
* **USART0**: Pins 0 (`PD0` / RXD) and 1 (`PD1` / TXD), 57600 baud, 8N1 (`uart`).
* **TWI / I2C**: Pins A4 (`PC4` / SDA) and A5 (`PC5` / SCL), 400 kHz Fast Mode (`twi`), controlling external PCA9685 (`pca9685`).
* **Timer2 (8-bit CTC)**: 1.0 ms system tick timebase (`timer_millis()`) and 50 Hz motion clock (`timer`).
* **Watchdog (WDT)**: 2.0-second hardware reset timeout.

---

## 3. Binary Protocol Specification

### 3.1 Wire Frame Anatomy
```
+--------+--------+--------+--------------------------+--------+
|  SYNC  | MSG_ID | LENGTH |         PAYLOAD          | CRC-8  |
| (0xAA) | (1 B)  | (1 B)  |        (0..16 B)         | (1 B)  |
+--------+--------+--------+--------------------------+--------+
         |<--------------- CRC-8 Scope -------------->|
```

* **SYNC**: `0xAA` (170 dec).
* **MSG_ID**: 1 byte message identifier.
* **LENGTH**: 1 byte payload length (0 to 16 bytes).
* **PAYLOAD**: 0 to 16 bytes of command/response arguments.
* **CRC-8**: 1 byte checksum calculated with polynomial $x^8 + x^2 + x^1 + x^0$ (`0x07`, ATM/SMBus) over `[MSG_ID, LENGTH, PAYLOAD...]`.

### 3.2 Inbound Command Registry (PC $\to$ Robot, 0x01..0x7F)

| Command Name | ID | Payload Length | Payload Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `CMD_PING` | `0x01` | 0 B | None | Heartbeat connectivity query. |
| `CMD_STOP` | `0x02` | 0 B | None | Emergency stop / abort motion. |
| `CMD_SET_SPEED` | `0x03` | 1 B | `[speed (1..10)]` | Set global speed scaling factor. |
| `CMD_GAIT_STEP` | `0x10` | 2 B | `[gait_id, steps]` | Execute gait (0=Sit, 1=Stand, 2=Fwd, 3=Back, 4=TurnL, 5=TurnR, 6=Wave). |
| `CMD_SET_LEG_SITE` | `0x11` | 7 B | `[leg, x_hi, x_lo, y_hi, y_lo, z_hi, z_lo]` | Direct Cartesian target (offloaded to PC compiler). |
| `CMD_SET_LEG_ANGLES` | `0x12` | 4 B | `[leg, coxa, femur, tibia]` | Direct joint angles in degrees (0..180). |
| `CMD_QUEUE_KEYFRAME` | `0x20` | 9 B | `[leg, coxa_hi, coxa_lo, femur_hi, femur_lo, tibia_hi, tibia_lo, dur_hi, dur_lo]` | Enqueue joint keyframe (tenths of deg) into static memory pool. |
| `CMD_CLEAR_QUEUE` | `0x21` | 0 B | None | Clear trajectory queue and release memory blocks. |
| `CMD_GET_STATUS` | `0x30` | 0 B | None | Request consolidated status packet. |
| `CMD_RESET_ERRORS` | `0x31` | 0 B | None | Clear error flags. |
| `CMD_GET_IMU` | `0x32` | 0 B | None | Request 6-DOF IMU attitude and acceleration telemetry. |

### 3.3 Outbound Response Registry (Robot $\to$ PC, 0x80..0xFF)

| Response Name | ID | Payload Length | Payload Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `RESP_ACK` | `0x80` | 1 B | `[cmd_id]` | Positive command execution acknowledgment. |
| `RESP_NACK` | `0x81` | 2 B | `[cmd_id, err_code]` | Negative acknowledgment / command rejection. |
| `RESP_PONG` | `0x84` | 0 B | None | Heartbeat keepalive response. |
| `RESP_STATUS` | `0x90` | 7 B | `[state, gait, queue_cnt, pool_avail, err, attached, speed]` | Telemetry status frame. |
| `RESP_IMU` | `0x91` | 12 B | `[roll_hi, roll_lo, pitch_hi, pitch_lo, ax_hi, ax_lo, ay_hi, ay_lo, az_hi, az_lo, gz_hi, gz_lo]` | 6-DOF IMU attitude and acceleration telemetry frame. |

### 3.4 Error Codes (`protocol_err_code_t`)
* `0x00` (`ERR_NONE`): Success.
* `0x01` (`ERR_BAD_CRC`): CRC-8 validation failed.
* `0x02` (`ERR_BAD_LENGTH`): Payload length out of bounds.
* `0x03` (`ERR_UNKNOWN_CMD`): Unrecognized message ID.
* `0x04` (`ERR_INVALID_PARAM`): Invalid parameters (e.g. invalid leg ID).
* `0x05` (`ERR_POOL_EXHAUSTED`): Static memory pool exhausted (all blocks in use).
* `0x06` (`ERR_QUEUE_FULL`): Motion trajectory queue full.
* `0x07` (`ERR_KINEMATIC_LIMIT`): Requested coordinate outside mechanical workspace.
* `0x08` (`ERR_BUSY`): Controller busy executing non-interruptible gait.

---

## 4. Host Motion Plan Compiler (`tetrapoder`)

Motion planning, kinematics compilation, and streaming are implemented in the standalone ATS tool **`tetrapoder`** (`/data/dev/python/3_tools/tetrapoder`):

```python
from tetrapoder.core.model.motion_plan import MotionPlan
from tetrapoder.core.service.compiler.motion_compiler import MotionCompiler
from tetrapoder.core.service.gait.gait_plan_builder import GaitPlanBuilder
from tetrapoder.core.service.runner.motion_runner import MotionRunner
from tetrapoder.core.service.trajectory.choreography_builder import ChoreographyBuilder
from tetrapoder.core.service.trajectory.trajectory_builder import TrajectoryBuilder
from tetrapoder.infrastructure.transport.serial_transport import SerialTransport

commands = []
choreography = ChoreographyBuilder(commands=commands)
gait = GaitPlanBuilder(commands=commands)
trajectory = TrajectoryBuilder(commands=commands)

# 1. Define choreography using high-level gaits
choreography.set_speed(speed=5)
gait.stand()
gait.walk(steps=4)
gait.turn(steps=2, to_right=False)
choreography.action_wave(count=2)
gait.sit()

# 2. Compile to binary wire stream
plan = MotionPlan(commands=tuple(commands))
binary_stream: bytes = MotionCompiler.compile(plan=plan)

# 3. Stream over serial to robot
transport = SerialTransport(port='/dev/ttyUSB0', baudrate=57600)
runner = MotionRunner(transport=transport)
runner.execute_plan(plan=plan)
```

Alternatively, plans can be compiled and streamed directly via the `tetrapoder` CLI:

```bash
# Compile gait into binary file
./run.sh robot --action compile --gait crawl --output motion_crawl.bin

# Compile and upload to robot controller
./run.sh robot --action upload --gait crawl --port /dev/ttyUSB0
```

