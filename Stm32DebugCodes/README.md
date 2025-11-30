
## STM32 debug projects

This folder contains small, focused STM32H7 projects used to debug and test
different peripherals, communication protocols, and sensors on the
NUCLEO‑H755ZI‑Q board.

Each subfolder is a self‑contained STM32CubeIDE project generated from a
`.ioc` file. Typical use cases are:

- verifying CAN bus configuration and communication
- testing IMU / MPU sensors over I²C / SPI
- checking basic pinout and GPIO behaviour
- experimenting with Bluetooth modules (HC‑05)

### Projects in this folder

- `CanMpu/` – CAN + MPU/IMU test project on STM32H755 (dual‑core CM4/CM7)
- `CanTest02/` – additional CAN bus test project
- `HC-05_AT/` – HC‑05 Bluetooth module AT‑command test project
- `PinTest/` – simple pin / GPIO test project

> All of these are intended as **debug / reference projects**, not
> production firmware.

---

## Using the CANable interface

The CAN‑related projects (`CanMpu`, `CanTest02`) are typically tested with a
CANable (slcan) USB‑to‑CAN adapter on Linux.

Example sequence to bring up the CANable as `slcan0` at 500 kbit/s:

```bash
# Attach CANable as SLCAN device (adapt <CAN_PORT>, e.g. /dev/ttyACM0)
sudo slcand -o -c -s6 <CAN_PORT> slcan0

# Bring interface up
sudo ip link set slcan0 up

# (Optional) inspect interface details
ip -details link show slcan0

# Monitor CAN traffic
candump slcan0
```

`-s6` corresponds to 500 kbit/s in slcan’s preset bit‑timing table, which
matches the default configuration used in `CanMpu` and `CanTest02`.

---

## Timer / ESC test notes

Some debug projects use a timer output to drive or emulate an RC ESC signal
(servo‑style PWM):

- ESC control signal: 50 Hz PWM
- Timer base clock: 200 Hz (example used in early tests)
- Prescaler example: `psc = 200 - 1` to derive 1 Hz tick from 200 Hz clock

These values are **example notes**, not strict requirements; always check the
timer configuration inside the specific CubeMX `.ioc` file and `Core/Src/*`
code for the exact setup used in a given project.

---

Refer to the root `README.md` for the overall tractor navigation project
context and how these debug projects relate to the main firmware.