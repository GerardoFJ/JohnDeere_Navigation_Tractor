# JohnDeere Navigation Tractor

Embedded navigation system for an Ackermann‑steered robot (John Deere‑style
tractor prototype) that follows waypoints using odometry and vision.

The platform is built around:

- **STM32H755** dual‑core MCU (NUCLEO‑H755ZI‑Q) for low‑level control and CAN
	communication
- **ESP32** for high‑level logic, networking, and user interaction
- **Ackermann car robot** chassis (steering + drive) (Servo motor and Esc Module)
- **CAN bus** between STM32 and ESP32
- **UART** links for debugging and module configuration
- **Positon camera** providing the robot’s pose estimate in the field

The goal is to drive the robot through a sequence of waypoints with closed‑loop
control that fuses odometry and camera‑based position.

---

## High‑level architecture

Conceptually, the system is split into three layers:

1. **Perception & planning (ESP32 + camera)**
	 - Reads wheel encoders and IMUs to compute odometry.
	 - Uses a camera‑based localization algorithm to estimate $(x, y, \theta)$ of
		 the robot in a fixed reference frame.
	 - Sends compact control messages over **CAN** to the STM32.

2. **Vehicle control (STM32H7)**
     - Receives or computes global waypoints.
     - Plans a path / steering command to drive from the current pose to the next
		 waypoint.
	 - Runs real‑time control loops for steering and throttle on the Ackermann
		 chassis.
	 - Exposes status (odometry, sensor data, debug info) over Bluetooth UART.

3. **Debug & tools (PC + CANable / UART adapters)**
	 - `Stm32DebugCodes/` contains focused STM32 projects for testing CAN, IMUs,
		 pins, and Bluetooth modules.
	 - `Esp32_Code/` contains the ESP32 firmware used for high‑level control and
		 communication with STM32.

---

## Repository layout

Top‑level folders:

- `Navigation_Tractor/` – Main STM32H7 firmware for the tractor navigation
	system handling waypoint logic and control (dual‑core CM4/CM7, generated with STM32CubeMX/STM32CubeIDE).
- `Esp32_Code/` – ESP32 side of the system (PlatformIO project) handling
	camera integration, and CAN communication with the STM32.
- `Stm32DebugCodes/` – Collection of small STM32 projects used to debug/test
	specific peripherals (CAN, IMUs, UART/Bluetooth, pinouts, etc.).

See the individual READMEs inside each folder for details.

---

## Key features

- **Waypoint navigation**
	- Follows a list of target poses in order.
	- Uses odometry to integrate motion between camera updates.
	- Uses camera pose to correct drift and maintain absolute accuracy.

- **Ackermann steering control**
	- Steering and motor control tuned for an Ackermann car‑style robot.
	- Stm32 calculations target steering angle and speed.

- **CAN‑based STM32–ESP32 link**
	- FDCAN peripheral on STM32 and CAN driver on ESP32.
	- Compact, binary message format for sensor data.

- **UART interfaces**
	- Debug console output (logs, diagnostics).
	- AT‑command interface to modules such as the **HC‑05**.

---

## Development environment

**MCU side (STM32H755):**

- Generated/configured with **STM32CubeMX**.
- Built and flashed using **STM32CubeIDE** (projects in `Navigation_Tractor/`
	and `Stm32DebugCodes/`).

**ESP32 side:**

- Implemented as a **PlatformIO** project (see `Esp32_Code/platformio.ini`).
- Can be developed in Platform IO CLI.

---

## Typical workflow

1. **Bring up and test the STM32 hardware**
	 - Open the relevant project under `Stm32DebugCodes/` in STM32CubeIDE.
	 - Flash and verify that CAN, UART, and sensors work as expected.

2. **Flash the main STM32 firmware**
	 - Open `Navigation_Tractor/Navigation_Tractor.ioc` in STM32CubeIDE.
	 - Regenerate code if needed and build for CM4/CM7.
	 - Flash to the NUCLEO‑H755ZI‑Q.

3. **Build and flash the ESP32 firmware**
	 - Open `Esp32_Code/` with PlatformIO.
	 - Select the correct environment/board.
	 - Build and upload the firmware to the ESP32.

4. **Connect the system**
	 - Wire the **CAN bus** between STM32 and ESP32 (proper termination,
		 common ground).
	 - Connect any necessary UART adapters and the camera.

5. **Run waypoint navigation**
	 - Start the vision / camera pipeline (on ESP32 or a companion processor,
		 depending on your setup).
	 - Provide a waypoint list.
	 - Monitor Bluetooth UART logs while the robot drives.

---

## Debugging and tools

- For STM32‑only experiments or low‑level protocol testing, use the projects
	under `Stm32DebugCodes/` (see their README for CANable commands and timer
	notes).
- To sniff CAN traffic on a PC, connect a CANable (or similar) adapter and use
	standard Linux tools like `slcand`, `ip link`, and `candump`.

---

## Status

This repository is a **work‑in‑progress research / prototype project**.
Interfaces and message formats may evolve while experimenting with different
controllers, sensors, and control laws.

Contributions, bug reports, and ideas for improving the navigation, control,
or infrastructure are welcome.



