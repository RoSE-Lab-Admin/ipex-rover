# Roxey motion primitive reference

All primitives run through one action, `/ipex/primitive`, served by `primitive_server`. How to call them (terminal, mission YAML, Python, ROS action): [`../ipex_tools/README.md`](../ipex_tools/README.md).

## What every primitive returns
| Field | Meaning |
|---|---|
| `success`, `failure_code`, `message` | outcome (codes below) |
| `duration_s` | **t**: time from start to finish |
| `energy_j` | **E**: total measured energy drawn during the primitive |
| `energy_above_idle_j` | E minus (idle power measured just before the start × t), i.e. the cost of the motion itself |
| `idle_power_w` | the idle power used above (average over 2 s before start) |
| `groups` / `group_energy_j` | E split into `drive`, `rear_arm`, `front_arm` |
| `outputs_yaml` | primitive-specific results |

**What E includes:** drive motors (RoboClaw voltage × measured motor current) and each arm's drum and shoulder driver supply (measured V × I). **Not included:** Pi, Teensys, RoboClaw logic, sensors. E is integrated at 10 Hz, so primitives shorter than about 1 s are less precise.

## Primitives
Angles are SI (rad, rad/s). Every `*_rad` / `*_rad_s` parameter also accepts `*_deg` / `*_deg_s`. `arm` is `front`, `rear` or `both`.

| Primitive | Capable | Parameters (default) | Stops when | Fails when |
|---|---|---|---|---|
| `drive_straight` | yes | `distance_m` (required, − = backward), `speed_mps` (0.2) | distance reached (odometry) | `STALL`, `TIMEOUT`, odometry lost |
| `point_turn` | yes (stalls on high-grip ground) | `dpsi_rad` (required, + = left), `omega_rad_s` (0.3) | heading change reached (odometry) | `STALL`, `TIMEOUT` |
| `arc_turn` | yes | `r_arc_m` (required), `speed_mps` (0.2), `dpsi_rad` (required) | heading change reached | `STALL`, `TIMEOUT` |
| `set_arm` | yes | `arm` (front), `theta_rad` (required; +45° = top/home, −40° = lowest), `omega_rad_s` (0.5–5.625°/s; default 5.625, restored after the move) | within 0.5° of target (encoder) | `PRECONDITION` (not homed), `INVALID_PARAMS` (limits), `TIMEOUT` |
| `excavate` | yes (time-based) | `arm` (front), `dig_angle_rad` (−10°) or `d_dig_m` (cut depth below the CAD tire bottom, 0–0.064 m; sets the angle), `omega_drum_rad_s` (drum rad/s; default 2000 steps/s = 3.75 drum RPM; also `drum_steps_s`, `drum_rpm`; max 3200 steps/s = 6 drum RPM), `speed_mps` (0.05 crawl), `duration_s` (10) | duration elapsed | `PRECONDITION`, `STALL`, `TIMEOUT` |
| `bulldoze` | yes (time- or distance-based) | `arm` (front), `blade_angle_rad` (−26.5° = drum ~1 cm above the CAD tire bottom) or `h_clear_m` (drum clearance above that plane; sets the angle; negative skims below it), `omega_drum_rad_s` (as excavate, but signed: + dig direction, − dump direction, 0 off), `speed_mps` (0.03), `duration_s` (10) or `distance_m` (odometry; overrides `duration_s`) | duration elapsed or distance reached | `PRECONDITION`, `STALL`, `TIMEOUT`, `HARDWARE` |
| `deposit` | yes (time-based) | `arm` (front), `dump_angle_rad` (45°), `omega_drum_rad_s` (as excavate), `speed_mps` (0), `duration_s` (10) | duration elapsed | `PRECONDITION`, `STALL`, `TIMEOUT` |
| `idle` | yes | `t_wait_s` (required) | t_wait elapsed | (only cancel / all stop / override) |
| `scan_terrain` | **no** | `region` | — | always `UNSUPPORTED` (no perception sensor) |
| `recharge` | **no** | `soc_target` | — | always `UNSUPPORTED` (no SoC estimate or docking) |

Accepted but not supported yet (return `UNSUPPORTED` if given): `excavate.m_target_kg`, `deposit.h_drop_m`, `deposit.m_deposit_kg`. These need load cells / drop-height kinematics; use the angle and `duration_s` parameters meanwhile.

**Cut depth** is measured from the **tire bottom plane in the CAD** (the lowest point of the wheels, grouser tips), not from the sand surface: depth = `drum_cut_radius_m` − (`arm_pivot_height_m` + `arm_length_m` · sin θ) = 0.149 − (0.3287 + 0.3786 sin θ) m. All three numbers are CAD only (2026-10-31): the shoulder pivot is 0.3183 m above `base_link` and the tire bottom is 10.4 mm below it (axle 0.1525 m, wheel mesh radius 0.163 m), so the pivot is 0.3287 m above the tire bottom; pivot to drum axle 0.3786 m (both arms); drum tooth-tip radius 0.149 m (drum meshes). The arm with drums reads exactly 45° at home, so the angle reference is good. On this plane the drum touches below −28.3° and the deepest cut at the −40° limit is 6.4 cm. Sinkage is **not** included: a rover sunk into soft ground cuts deeper than reported (on 2026-10-21, −35° reported 37 mm here and measured 53 mm). Compliance / sinkage corrections come later, with the arm strain gauges. `set_arm`, `excavate` and `bulldoze` report depth or clearance on this plane.

**Drum order:** `excavate` spins the drums up (`drum_spinup_s`, 1 s) before the arms lower, so they reach the ground already turning. Drums are always the last thing to stop: on any end except all stop (success, cancel, override, stall, timeout) the server halts the arms, stops the wheels, waits for odometry to show them stopped (max `drum_stop_wait_s`, 3 s) plus `drum_stop_delay_s` (0.5 s), then stops the drums. This applies to `bulldoze` and `deposit` too.

**Bulldoze** is a push, not a dig: the drum rides just above the ground and spins while the rover crawls slowly, so loose material gets pushed and churned ahead of it. Torque comes from the RoboClaw speed loop, which applies whatever current holding `speed_mps` takes, up to the RoboClaw current limit (set in Motion Studio). The `STALL` check (≥ 6.5 A for 1 s, `stall_current_a`) still applies, so a push that bogs down ends with `STALL`.

Drum direction: `omega_drum_rad_s > 0` digs on `excavate` and `bulldoze` (the hand controller's D-pad-right direction; `bulldoze` also takes negative for the dump direction); `deposit` always spins in the dump direction.

## Failure codes
| Code | Meaning |
|---|---|
| `INVALID_PARAMS` | unknown / missing / out-of-range parameter, or unknown primitive |
| `UNSUPPORTED` | primitive or parameter not implemented on this rover yet |
| `PRECONDITION` | e.g. arm not homed, no odometry, no arm angle feedback |
| `BUSY` | another primitive is running |
| `TIMEOUT` | did not finish within its time limit (expected time × 1.5 + margin) |
| `STALL` | a drive motor at ≥ 6.5 A (ACS712 sensor, `/rover/<wheel>/current`) for ≥ 1 s |
| `ESTOP` | all stop (B / `roxey stop` / `/ipex/all_stop`) |
| `OPERATOR_OVERRIDE` | operator used the hand controller during the primitive |
| `CANCELED` | the caller canceled (e.g. Ctrl+C) |
| `HARDWARE` | required telemetry lost mid-run |
| `INTERNAL` | software error (see the rover log) |

## Known limitations (current hardware)
- **Odometry is from the wheel encoders** (RoboClaw, since 2026-10-22): distance and heading are wheel travel, so slip and turn skid are not detected. There is no IMU.
- **Arm speed** is settable from 0.5 to 5.625 °/s; faster needs ``ARM_SPEED_MAX_DEG_S`` raised in firmware after checking for missed steps. Stall / torque-limit detection isn't possible from current, because steppers draw constant current; it needs encoder-vs-commanded checking.
- **No payload measurement:** `m_load`, `m_target`, `m_deposit` need arm load cells.

## Configuration
`ipex_bringup/config/primitives.yaml`: arm limits and tolerance, speed and turn caps, timeouts, stall threshold, drum direction signs, idle-power window, data directory.

## Adding a primitive
1. Subclass `Primitive` in `ipex_primitives/library.py`: set `NAME`, `SUMMARY`, `PARAMS`, `STOPS_WHEN`, `FAILS_WHEN`, and implement `run(ctx)`. Optionally implement `cleanup(ctx)`.
2. Use `ctx.sleep()` / `ctx.check()` in loops so cancel, all stop, override and timeouts work. Raise `PrimitiveFailure(Codes.X, msg)` for failures.
3. Add it to `CATALOG`. Timing, energy, logging, CLI, missions and the Python client pick it up automatically.
