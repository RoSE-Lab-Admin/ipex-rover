# Driving Roxey with motion primitives

Roxey exposes its capabilities as **motion primitives** (drive straight, turn, set arm, excavate, ...). Every primitive call returns **how long it took (t)** and **how much energy it used (E)**, plus success or a failure code. There are four ways to call them; pick the one that fits.

| You are... | Use | Example |
|---|---|---|
| An operator at a terminal | `roxey run` | `roxey run set_arm arm=front theta_deg=-10` |
| Planning a sequence | a **mission YAML** | `roxey mission my_test.yaml --record` |
| Writing Python (algorithms, DAGs) | `RoxeyClient` | `RoxeyClient().run('point_turn', dpsi_deg=90)` |
| Writing any ROS 2 code | the action directly | `/ipex/primitive` (`ipex_interfaces/action/Primitive`) |

Primitive names, parameters, units and failure codes: [`../ipex_primitives/README.md`](../ipex_primitives/README.md), or `roxey prims` / `roxey prims NAME` on the rover.

## Setup (on the rover's Pi)
```bash
source ~/ipex_ws/install/setup.bash
roxey prims            # works from any directory once the workspace is sourced
```
The rover service must be running (`sudo systemctl status ipex-rover`). `roxey` uses ROS domain 42 automatically.

## 1. `roxey run`: one primitive
```bash
roxey run set_arm arm=front theta_deg=-10
roxey run drive_straight distance_m=1.0 speed_mps=0.2 --record
roxey run idle t_wait_s=30 --tag baseline
```
- Parameters are `key=value`. Angles accept degrees (`theta_deg`, `dpsi_deg`, `omega_deg_s`); drum speed accepts `drum_steps_s`.
- Prints live progress, then **t, E, E above idle**, energy per subsystem, and outputs.
- `--record` saves a bag of the run (see Data). Ctrl+C cancels the primitive on the rover.

## 2. Mission files: a sequence of primitives
```yaml
name: dig_cycle                   # optional; defaults to the file name
description: Dig, back up, dump.
record: true                      # optional: bag for the whole mission
on_failure: stop                  # stop (default) or continue
steps:
  - set_arm: {arm: front, theta_deg: 0}                  # short form
  - primitive: excavate                                  # long form (adds a tag)
    tag: dig
    params: {arm: front, dig_angle_deg: -10, duration_s: 10}
  - drive_straight: {distance_m: -0.5, speed_mps: 0.15}
  - deposit: {arm: front, duration_s: 10}
```
```bash
roxey mission my_test.yaml --check     # validate only, nothing moves
roxey mission my_test.yaml             # run
roxey mission dig_cycle --record       # installed example, with a bag
```
The whole file is validated before anything moves (unknown primitives, bad or missing parameters, unsupported features). The output is a per-step table plus mission totals (time, energy, energy above idle).

Installed examples: `idle_baseline`, `arm_check`, `dig_cycle`, `drive_square`.

## 3. Python: `RoxeyClient`
```python
import rclpy
from ipex_tools.client import RoxeyClient

rclpy.init()
roxey = RoxeyClient()

r = roxey.run('set_arm', arm='front', theta_deg=-10)
print(r.success, r.failure_code, r.duration_s, r.energy_j, r.energy_above_idle_j)
print(r.groups)    # {'drive': ..., 'rear_arm': ..., 'front_arm': ...}  (J)
print(r.outputs)   # primitive-specific, e.g. {'final_front_arm_deg': -9.8}

if not r.success and r.failure_code == 'STALL':
    roxey.run('drive_straight', distance_m=-0.2)   # e.g. a DAG branch

roxey.close()
rclpy.shutdown()
```
`run()` blocks until the primitive finishes. It returns `failure_code='BUSY'` if another primitive is running.

## 4. ROS 2 action
`/ipex/primitive`, type `ipex_interfaces/action/Primitive`:
- **Goal:** `name`, `params_yaml` (YAML mapping as text, e.g. `"{arm: front, theta_deg: -10}"`), `tag`.
- **Result:** `success`, `failure_code`, `message`, `duration_s`, `energy_j`, `energy_above_idle_j`, `idle_power_w`, `groups` / `group_energy_j`, `outputs_yaml`.
- **Feedback:** `elapsed_s`, `energy_j`, `progress`, `status`.

## Safety
- **B on the hand controller** (or `roxey stop`) is an all stop: every Teensy stops, and any running primitive ends with `ESTOP`.
- **Using the hand controller** (sticks, arm buttons, drum D-pad) during a primitive cancels it with `OPERATOR_OVERRIDE`. The operator always wins.
- Only one primitive runs at a time.
- Arm primitives require homed arms: **A** on the controller, or `roxey home` (**the arms move**).

## Data
Everything goes under `~/roxey_data/` on the Pi:

| Path | Contents |
|---|---|
| `runs/primitive_runs.csv` | every primitive ever run, from any interface: params, result, t, E, breakdown |
| `missions/<time>_<name>/` | `report.yaml` + `steps.csv` per mission, plus `bag/` if recorded |
| `recordings/<time>_<name>/` | `bag/` + `notes.yaml` (git commit, notes) from `roxey record` / `run --record` |

`roxey runs` shows the most recent results. Copy data to another computer with `scp -r ipex-rp5@<pi-ip>:~/roxey_data .`, and open bags in Foxglove (Open local file → the `.mcap` inside `bag/`).

## Other commands
| Command | Does |
|---|---|
| `roxey energy` | live power and energy per channel and subsystem (`--once`, `--reset`) |
| `roxey record NAME [--duration S] [--notes "..."]` | record a bag (Ctrl+C to stop) |
| `roxey home` | home both arms (**arms move**) |
| `roxey stop` | all stop |
