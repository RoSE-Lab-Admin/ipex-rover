# Firmware

Teensy firmware for Roxey. Each board talks to `ipex_hardware` over USB serial using a line-based ASCII protocol. Firmware and `ipex_hardware` change together; document protocol changes in `docs/`.

| Folder | Board | Serial ID (`/dev/serial/by-id/...`) |
|---|---|---|
| `drivetrain/` | Drivetrain Teensy | `usb-Teensyduino_USB_Serial_20406990-if00` |
| `arm/` | Front + rear arm Teensys (same firmware) | Rear: TBD, Front: TBD |

## Status

- `STOP` is accepted at any time on every board.
- Drivetrain stops motors if no `DRIVE` for 500 ms. Arm drums are continuous (no timeout) by design.
- Arm homing timeout: TODO once full-travel time is measured.
