# Getting started

## Requirements

### Option 1: Docker (Recommended)
The easiest way to get started is using Docker, which includes all dependencies:

```bash
docker build -t ramp-autonomy .
docker run --rm -it ramp-autonomy
```

For GUI mode:
```bash
docker run --rm -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ramp-autonomy --gui
```

### Option 2: Native Installation
ROS2 Jazzy, Gazebo gz-sim 8, and Nav2 (`nav2_controller`, `nav2_lifecycle_manager`, `nav2_velocity_smoother`, `robot_localization`). The MPPI
critic plugins in `ramp_autonomy_critics` additionally need `libxtensor-dev`.

## Build

### Using Docker (Recommended)

The Docker container is pre-built with all dependencies and the workspace already compiled:

```bash
docker build -t ramp-autonomy .
```

### Native Build

```bash
cd ramp_autonomy_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note:** If building natively, ensure you have ROS 2 Jazzy and all dependencies installed first.

## Run

### Using Docker (Recommended)

Headless mode (default, no GUI):
```bash
docker run --rm -it ramp-autonomy
```

With GUI support:
```bash
xhost +local:docker  # Allow Docker to connect to X server
docker run --rm -it --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ramp-autonomy --gui
```

### Native Run

```bash
# unloaded run, sharp-transition ramp world
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=sharp

# loaded run, payload offset 0.1 m forward, curved-transition world
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=curved payload_mass:=100.0 payload_cg_offset:=0.1

# the realistic construction site (the primary test environment since phase 8)
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=100.0
```

### Performance Measurement

To measure the real-time factor (RTF) inside the container:

```bash
docker run --rm -it ramp-autonomy /workspace/ramp_autonomy_ws/src/ramp_autonomy_control/scripts/measure_rtf.py
```

**Note:** RTF may be lower inside the container compared to native execution. The harness requires RTF ≥ 0.95 for validation runs. If your container RTF is below this threshold, validation tests may fail even if they would pass natively.

Watch `/ramp_state`, `/stability_margin`, and `/wheel_slip` with `ros2 topic echo` or RViz to see the state machine react as the robot crosses a
ramp.

If the Gazebo GUI client crashes on your setup (a `libpthread.so.0: undefined symbol: __libc_pthread_init` symbol clash from a snap-packaged GTK
library on `LD_LIBRARY_PATH` is the specific case this project hit), add `headless:=true`. Physics, sensors, and every topic/bridge work
identically either way - you just don't get a 3D view. `visualization/run_live_gui.sh` wraps the environment fix (`unset GTK_PATH
GTK_EXE_PREFIX`) that resolves it when the GUI is needed.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `world` | `sharp` | `sharp`, `curved`, or `construction_site` |
| `payload_mass` | `0.0` | kg added to the robot; `0.0` renders no payload |
| `payload_cg_offset` | `0.0` | forward/back offset of the payload's center of gravity, metres |
| `detachable_payload` | `false` | `true` spawns the payload as a separate model held by a fixed joint, released at the delivery pad |
| `cargo_mode` | `secured` | `loose` makes the payload a free rigid body from spawn - a demo mode, never used by any validated test; see `results.md` |
| `spawn_x`/`spawn_y`/`spawn_z` | `1.0`/`0.0`/`0.23` | robot spawn pose |
| `headless` | `false` | `true` runs the Gazebo server only, no GUI |

## Running the validation suite

The scripts that produced the numbers in `results.md` live under `validation/phase8/`. They resolve the workspace root from the `RAMP_WS`
environment variable, so export it once before running any of them:

```bash
export RAMP_WS=/path/to/ramp_autonomy_ws
```

A script run without `RAMP_WS` set exits immediately with a message rather than silently reading the wrong path.

Each session's own `README.md` in that tree documents its exact invocation; `validation/phase8/harness/harness_guard.py` is the shared clean-host guard (process cleanup, a pre-run real-time-factor
check, a health monitor during the run) every guarded script calls before and after driving the robot - see `validation-methodology.md` for why.

A representative example, the ground-truth-gated construction-site round trip:

```bash
cd validation/phase8/construction_site_descent
python3 arc_turn_followpath_test_ekf.py <out.csv>
```

### If the guard refuses to start

`harness_guard.py` enforces a hard real-time-factor floor of 0.95 before any guarded trial. The construction-site world no longer clears that
on ordinary hardware - roughly 0.90-0.92 headless and 0.70 with the Gazebo GUI up on a 12-core laptop with an RTX 3050 - so a run there can
abort immediately with:

```
!! HARNESS GUARD FAILED - DO NOT RUN THE TRIAL
!! real-time factor 0.91 < 0.95 (simulator cannot keep real time)
```

That is the guard working, not a broken checkout. `HG_LOW_RTF` will not get you past it (it relaxes a different, in-run check). Running
`headless:=true` helps but is unlikely to be enough on its own. See `known-limitations.md` for the full picture - and if you hit this, please
open an issue saying what real-time factor your machine sustains, because how widespread this is across hardware is not yet known.

### A note on the recorded data

This repository tracks the harness, the analysis code and the written-up results, but not the several GB of raw per-run output the suite
produces (rosbags, per-topic CSV/JSON traces, node logs, screen recordings). All of it is regenerated by re-running the scripts above. The
round-trip videos referenced from the README are attached to the GitHub release instead of committed.
