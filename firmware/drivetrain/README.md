# Drivetrain Teensy

Serial: `/dev/serial/by-id/usb-Teensyduino_USB_Serial_20406990-if00`, 115200 baud.

## Protocol (as used by `ipex_hardware/SerialTransport`)

| Direction | Line | Meaning |
|---|---|---|
| Pi → Teensy | `DRIVE <left_rad_s> <right_rad_s>` | Side wheel velocities |
| Teensy → Pi | `VOLT <bl> <br> <fr> <fl>` | Motor voltages |
| Teensy → Pi | `CURR <bl> <br> <fr> <fl>` | Motor currents |
| Teensy → Pi | `CAL_CURRENT START` / `CAL_CURRENT OK` | Current-sensor calibration status (ignored by the Pi) |

Source code: copy here.
