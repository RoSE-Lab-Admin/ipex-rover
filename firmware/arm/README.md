# Arm Teensy (front and rear)

One board per arm; both run `arm.ino`. Each controls its arm's shoulder (two steppers: one with encoder, one with brake, driven with the same step pulses in opposite directions) and its two drum motors (driven in sync from one step signal).

| Arm | Serial ID |
|---|---|
| Rear | `usb-Teensyduino_USB_Serial_19972490-if00` (legacy name: back_implement) |
| Front | `usb-Teensyduino_USB_Serial_20404840-if00` |

Libraries: `Encoder`, `AccelStepper` (both ship with Teensyduino).

## Behavior
- Movement sequence is the original sketch's, plus a settle delay: release brake (200 ms) → step both motors (800 µs half-period, 400 steps/rev, 100:1) → settle (50 ms) → engage brake (200 ms).
- **Interruptible:** the step loops check serial before every pulse. `STOP` or a new `ARM` exits the loop between pulses (step pins LOW), then settle → engage, exactly like the end of a normal move. A new `ARM` then runs as a fresh move (release → step → settle → engage).
- **Brake interlock:** `armStepPulse()` refuses unless the brake has been released ≥ 200 ms; `engageBrake()` refuses while any step loop is running.
- Homing angle: `HOME_ANGLE_DEG` (+45) is the only value to change if the homing switch moves.
- Arm motion is **open loop**: the encoder is read to plan each move.
- Travel clamped to **-45 to +45 deg**. After homing, the arm is set to `HOME_ANGLE_DEG` (**confirm the physical angle at the limit switch before flashing**).
- Drums are continuous (no timeout); `DRUM` applies immediately without interrupting the arm. While the arm is stepping, drum steps are limited to one per arm pulse (same as original).

## Protocol (115200, newline-terminated, case-insensitive)

| Direction | Line | Meaning |
|---|---|---|
| Pi → Teensy | `CAL` | Home to limit switch (only when idle) |
| Pi → Teensy | `ARM <deg>` | Absolute target; interrupts an in-progress move |
| Pi → Teensy | `DRUM <steps_s>` | Drum speed, ±3200 |
| Pi → Teensy | `STOP` | Stop arm, engage brake, ramp drums to 0 |
| Teensy → Pi | `CAL_START` / `CAL_DONE` / `CAL_ABORTED` | Homing status |
| Teensy → Pi | `ARM_DONE <deg>` | Move complete, brake engaged |
| Teensy → Pi | `STOPPED` | STOP complete, brake engaged |
| Teensy → Pi | `STATE <homed> <arm_deg> <IDLE\|MOVING\|HOMING> <drum_steps_s>` | 10 Hz, also during moves |
| Teensy → Pi | `ERR NOT_HOMED` / `ERR BUSY` / `ERR UNKNOWN_COMMAND` | Rejected command |
| Teensy → Pi | `# <text>` | Info; Pi ignores |

## Not yet implemented
- Homing timeout (add once full-travel time is measured).
- Closed-loop arm control from the encoder.

Original sketch: `legacy/sketch_sep22_da_integration.ino.orig`.
