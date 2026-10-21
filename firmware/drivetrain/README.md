# Drivetrain Teensy

Serial: `/dev/serial/by-id/usb-Teensyduino_USB_Serial_20406990-if00`, 115200 baud.

Drives both RoboClaws (velocity PID speed commands) and reads all power sensors: drive motor currents, and current + voltage for each arm's drum and shoulder driver supply.

## Protocol (as used by `ipex_hardware/SerialTransport`)

| Direction | Line | Meaning |
|---|---|---|
| Pi → Teensy | `DRIVE <left_rad_s> <right_rad_s>` | Side wheel velocities (stops after 500 ms without a new one) |
| Pi → Teensy | `STOP` | Stop all drive motors |
| Pi → Teensy | `CAL_CURRENT` | Zero the drive current sensors (sensors powered, motors still); **saved to EEPROM** |
| Pi → Teensy | `CAL_ARM_CURRENT` | Zero the arm current sensors (**arm driver supply leads disconnected**); **saved to EEPROM** |
| Pi → Teensy | `CAL_SHOW` | Print the drive and arm zeros currently in use |
| Teensy → Pi | `VOLT <bl> <br> <fr> <fl>` | Drive motor voltage estimate (RoboClaw battery V × PWM duty) |
| Teensy → Pi | `CURR <bl> <br> <fr> <fl>` | Drive motor currents (A); ACS712 30A, stored zeros, no boot calibration |
| Teensy → Pi | `ARM_CURR <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>` | Arm driver supply currents (A), 10 Hz |
| Teensy → Pi | `ARM_VOLT <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>` | Arm driver supply voltages (V), 10 Hz |
| Teensy → Pi | `BATT <drivetrain_V> <pi_arm_V>` | Battery voltages (V), 10 Hz |
| Teensy → Pi | `RC_CURR <bl> <br> <fr> <fl>` | Motor currents reported by the RoboClaws (A, signed; `nan` if no reply), 10 Hz. **Unreliable (offset at standstill): use `CURR`.** |
| Teensy → Pi | `SPEED <bl> <br> <fr> <fl>` | Measured wheel speed from the RoboClaw encoders (wheel rad/s, + = forward, same frame as `DRIVE`; `nan` if no reply), 10 Hz |
| Teensy → Pi | `ENC <bl> <br> <fr> <fl>` | Wheel position from the RoboClaw encoders (rad since RoboClaw power-up, same frame), 10 Hz |
| Teensy → Pi | `CAL_CURRENT OK SAVED <z0..z3>` / `CAL_CURRENT REJECTED ...` | Drive zeros saved (sensor-side volts, bl br fr fl), or rejected because a channel looked unpowered (outside 2.2-2.8 V) |
| Teensy → Pi | `CAL_ARM_CURRENT OK SAVED <z0..z3>` / `... REJECTED ...` | Arm zeros saved, or rejected |
| Teensy → Pi | `STOPPED` | STOP complete |

## Arm power sensors

Current sensors (ACS712, per chip marking): **drums 30 A (0.066 V/A), shoulders 5 A (0.185 V/A, saturates at 5 A)**. Voltage: DC 0-25V module (5:1) + 5 V to 3.3 V divider. Drive motor sensors (pins 20-23): **all 30 A** (verified 2026-10-18).

| Pin | Signal | ROS topic |
|---|---|---|
| 16 | Rear drum current | `/rover/rear_arm/drum/current` |
| 17 | Rear drum voltage | `/rover/rear_arm/drum/voltage` |
| 18 | Rear shoulder current | `/rover/rear_arm/shoulder/current` |
| 19 | Rear shoulder voltage | `/rover/rear_arm/shoulder/voltage` |
| 38 | Front shoulder voltage | `/rover/front_arm/shoulder/voltage` |
| 39 | Front shoulder current | `/rover/front_arm/shoulder/current` |
| 40 | Front drum voltage | `/rover/front_arm/drum/voltage` |
| 41 | Front drum current | `/rover/front_arm/drum/current` |

**Teensy analog pins are 3.3 V max.** Every module output goes through a 5 V to 3.3 V divider (ratio 1.5). Voltage channels: measured ratio on the rover **9.10:1** (27.77 V in, 3.05 V at the pin). A full 7S pack (29.4 V) gives ~3.23 V at the pin (just under 3.3 V); readings clip at ~30 V. A 15k bottom resistor (instead of 20k) would give ~10.3:1 and more margin.

## Battery voltage sensors

Same 0-25V module + divider as the arm voltage channels, inline on each battery.

| Pin | Signal | ROS topic |
|---|---|---|
| 24 | Drivetrain battery voltage | `/rover/drive_battery/voltage` |
| 25 | Pi / arm battery voltage | `/rover/arm_battery/voltage` |

Ratio set to 9.10 (same sensor + divider as the arm channels); verify against a multimeter once wired (`BATTERY_VOLTAGE_DIVIDER_RATIO`).

## Current-sensor calibration

Zeros are saved in the Teensy EEPROM by `CAL_CURRENT` / `CAL_ARM_CURRENT` and loaded at every boot (no reflash needed). If nothing is saved, compiled defaults are used (arm: 2026-10-15 manual values; drive: nominal 2.5 V). There is **no boot auto-zero**. A calibration is rejected if any channel reads outside 2.2-2.8 V (sensor unpowered or current flowing). `CAL_SHOW` prints what is in use.

- Drive: sensors powered, wheels still, send `CAL_CURRENT`.
- Arms: disconnect each arm driver supply lead downstream of its sensor, send `CAL_ARM_CURRENT`.
