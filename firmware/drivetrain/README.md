# Drivetrain Teensy

Serial: `/dev/serial/by-id/usb-Teensyduino_USB_Serial_20406990-if00`, 115200 baud.

Drives both RoboClaws (velocity PID speed commands) and reads all power sensors: drive motor currents, and current + voltage for each arm's drum and shoulder driver supply.

## Protocol (as used by `ipex_hardware/SerialTransport`)

| Direction | Line | Meaning |
|---|---|---|
| Pi → Teensy | `DRIVE <left_rad_s> <right_rad_s>` | Side wheel velocities (stops after 500 ms without a new one) |
| Pi → Teensy | `STOP` | Stop all drive motors |
| Pi → Teensy | `CAL_CURRENT` | Zero the drive current sensors (sensors powered, motors still); prints zeros to paste into `current_zero_offsets` |
| Pi → Teensy | `CAL_ARM_CURRENT` | Zero the arm current sensors (**arm driver supply leads disconnected**) |
| Teensy → Pi | `VOLT <bl> <br> <fr> <fl>` | Drive motor voltage estimate (RoboClaw battery V × PWM duty) |
| Teensy → Pi | `CURR <bl> <br> <fr> <fl>` | Drive motor currents (A); ACS712 30A, stored zeros, no boot calibration |
| Teensy → Pi | `ARM_CURR <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>` | Arm driver supply currents (A), 10 Hz |
| Teensy → Pi | `ARM_VOLT <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>` | Arm driver supply voltages (V), 10 Hz |
| Teensy → Pi | `BATT <drivetrain_V> <pi_arm_V>` | Battery voltages (V), 10 Hz |
| Teensy → Pi | `RC_CURR <bl> <br> <fr> <fl>` | Motor currents measured by the RoboClaws (A, signed; `nan` if no reply), 10 Hz |
| Teensy → Pi | `CAL_CURRENT START` / `CAL_CURRENT OK <z0> <z1> <z2> <z3>` | Drive current zeros (sensor-side volts, order bl br fr fl) |
| Teensy → Pi | `CAL_ARM_CURRENT START` / `CAL_ARM_CURRENT OK <z0> <z1> <z2> <z3>` | Arm current zeros (sensor-side volts) to paste into `arm_current_zero_v` |
| Teensy → Pi | `STOPPED` | STOP complete |

## Arm power sensors

UMLIFE kit: ACS712 30A current module (0.066 V/A) + DC 0-25V voltage module (5:1 divider).

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

**Teensy analog pins are 3.3 V max.** Every module output goes through a 5 V to 3.3 V divider (ratio 1.5). Voltage channels: bench-measured total ratio **11.8:1** (not the 7.5 the nominal values suggest; module and divider load each other). A full 7S pack (29.4 V) gives ~2.5 V at the pin; full scale ~38.9 V. Trim each channel against a multimeter (`ARM_VOLTAGE_DIVIDER_RATIO`).

## Battery voltage sensors

Same 0-25V module + divider as the arm voltage channels, inline on each battery.

| Pin | Signal | ROS topic |
|---|---|---|
| 24 | Drivetrain battery voltage | `/rover/drive_battery/voltage` |
| 25 | Pi / arm battery voltage | `/rover/arm_battery/voltage` |

Ratio starts at 11.8; trim each channel against a multimeter (`BATTERY_VOLTAGE_DIVIDER_RATIO`).

Zero-current offsets are stored constants (manual calibration 2026-10-15); there is no boot auto-zero for the arm channels. To redo them, disconnect each arm driver supply lead downstream of its sensor, send `CAL_ARM_CURRENT`, and paste the printed values into `arm_current_zero_v`.
