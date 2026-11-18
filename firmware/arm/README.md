# Arm Teensy (front and rear)

One board per arm; both run `arm.ino`. Each controls its arm's shoulder (two steppers: one with encoder, one with brake, driven with the same step pulses in opposite directions) and its two drum motors (driven in sync from one step signal).

| Arm | Serial ID |
|---|---|
| Rear | `usb-Teensyduino_USB_Serial_20404840-if00` (rear and front swapped 2026-10-22) |
| Front | `usb-Teensyduino_USB_Serial_19972490-if00` |

Libraries: `Encoder`, `AccelStepper` (both ship with Teensyduino).

## Behavior
- Movement sequence is the original sketch's, plus a settle delay: release brake (200 ms) → step both motors (default 800 µs half-period = 5.625 °/s, settable with `ARM_SPEED`; 400 steps/rev, 100:1) → settle (50 ms) → engage brake (200 ms).
- **Interruptible:** the step loops check serial before every pulse. `STOP`, a new target, or a jog exits the loop between pulses (step pins LOW), then settle → engage, exactly like the end of a normal move. The next move/jog starts fresh (release → step → settle → engage).
- **Brake interlock (both apply in every mode, including hold):**
  1. **Never step with the brake engaged:** `armStepPulse()` refuses unless the brake was released ≥ 200 ms ago **and** the relay output reads back released right now.
  2. **Never engage the brake on a motor commanded to move:** `engageBrake()` refuses while any step loop (move, jog, homing, hold) is running or a step pin is HIGH, and waits 50 ms after the **last** step.
- **Hold mode** (depth hold while digging, added 2026-11-18): `HOLD ON` releases the brake **once** and the motors hold the arm at a target by closed loop on the motor encoder (step toward the target whenever it is ≥ 1 step away). `ARM`, `STEP` and `JOG` move the target with no brake cycle. Brake engages again (stop stepping → settle → engage, `HOLD_DONE <reason>`) on:
  - `HOLD OFF` (`RELEASED`), `STOP` (`STOP`), no `HOLD ON` keepalive for 500 ms (`TIMEOUT`);
  - **slip:** encoder differs from the commanded steps by > 1° (`SLIP`): the load back-drove or stalled the motors;
  - relay not driven to released, or a refused step (`FAULT`).
  After any end except `HOLD OFF` the hold is **latched**: `HOLD ON` is ignored until the Pi has stopped sending it for 500 ms, so a fault can't be followed straight away by another brake release. The 500 ms and 1° are chosen values, not specs. The spring brake engages on power loss or a Teensy reset. Not checked by the encoder: slip after the motor (gearbox / set screw).
- **Jog:** runs only while `JOG UP|DOWN` keepalives arrive (stops 250 ms after the last). Soft limits are checked before the brake is released and before every step; holding into a limit never cycles the brake. A jog stopped by limit or conflict stays latched until `JOG STOP` or a keepalive gap (button released).
- **Drums keep running during all arm activity** (every arm wait services the drum stepper). Drum step pulse width is 10 µs for the TB6600.
- Homing angle: `HOME_ANGLE_DEG` (+45) is the only value to change if the homing switch moves.
- Moves and jogs are **open loop** (the encoder plans each move and checks jog limits); hold mode is closed loop on the encoder.
- Travel limited to **-40 to +45 deg** (`ARM_MIN_DEG` / `ARM_MAX_DEG`).
- Drums are continuous (no timeout); `DRUM` applies immediately without interrupting the arm.

## Protocol (115200, newline-terminated, case-insensitive)

| Direction | Line | Meaning |
|---|---|---|
| Pi → Teensy | `CAL` | Home to limit switch (only when idle) |
| Pi → Teensy | `ARM <deg>` | Absolute target; interrupts a move/jog |
| Pi → Teensy | `STEP <deg>` | Relative target (taps); accumulates onto a move in progress |
| Pi → Teensy | `JOG UP` / `JOG DOWN` | Start or keep alive a jog (send ≤ every 250 ms) |
| Pi → Teensy | `JOG STOP` | End a jog |
| Pi → Teensy | `DRUM <steps_s>` | Drum speed, ±25600 (drum TB6600s at 1600 pulses/rev: 1 drum RPM = 2133.3 steps/s, max 12 RPM) |
| Pi → Teensy | `ARM_SPEED <deg_s>` | Arm speed for moves and jogs, clamped to 0.5–5.625 °/s (default 5.625 after reset); applies from the next step. Homing always uses the default. `ERR BUSY` while homing. |
| Pi → Teensy | `HOLD ON` | Enter / keep alive hold mode (send ≤ every 500 ms; the plugin sends every 100 ms). Refused (`ERR NOT_HOMED` / `ERR BUSY`) unless homed and not homing; ignored while latched |
| Pi → Teensy | `HOLD OFF` | Leave hold mode: stop, settle, brake |
| Pi → Teensy | `STOP` | Stop arm (then brake) and drums |
| Teensy → Pi | `CAL_START` / `CAL_DONE` / `CAL_ABORTED` | Homing status |
| Teensy → Pi | `ARM_DONE <deg>` | Move complete, brake engaged |
| Teensy → Pi | `JOG_DONE <reason> <deg>` | Jog ended, brake engaged. Reason: `RELEASED`, `TIMEOUT`, `LIMIT`, `CONFLICT`, `INTERRUPTED`, `FAULT` |
| Teensy → Pi | `JOG_LIMIT <deg>` | Jog refused at a limit (brake never released) |
| Teensy → Pi | `STOPPED` | STOP complete, brake engaged |
| Teensy → Pi | `HOLD_START <deg>` | Hold mode on, brake released |
| Teensy → Pi | `HOLD_DONE <reason> <deg>` | Hold mode over, brake engaged. Reason: `RELEASED`, `TIMEOUT`, `STOP`, `SLIP`, `FAULT` |
| Teensy → Pi | `ARM_SPEED <deg_s>` | Arm speed now in use (reply to `ARM_SPEED`) |
| Teensy → Pi | `STATE <homed> <arm_deg> <IDLE\|MOVING\|JOGGING\|HOMING\|HOLDING> <drum_steps_s>` | 10 Hz, also during moves |
| Teensy → Pi | `ERR NOT_HOMED` / `ERR BUSY` / `ERR UNKNOWN_COMMAND` / `ERR BRAKE_NOT_RELEASED` | Rejected command |
| Teensy → Pi | `# <text>` | Info; Pi ignores |

## Not yet implemented
- Homing timeout (add once full-travel time is measured).
- Closed-loop control for normal moves (hold mode has it).

Original sketch: `sketch_sep22_da_integration` (removed 2026-11-03; in git history as `firmware/arm/legacy/sketch_sep22_da_integration.ino.orig`).

Host simulation of the hold mode and both interlocks (fake Arduino, encoder follows the step pins): 21 checks, ~10,500 steps, 0 steps with the brake engaged, 0 brake engagements < 50 ms after a step (2026-11-18).
