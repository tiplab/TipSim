**English** | [简体中文](README.zh-CN.md)

# TipSim Samples

This guide covers the Python / C++ samples, build instructions, ROS2 publishing, 3D tactile visualization, and troubleshooting for this directory. To load the plugins, declare sensors, and read `sensordata` in your own project, see the root [README.md](../README.md).

## Requirements

Running or building the samples requires the following dependencies in addition to the plugin runtime requirements:

> **Plugin compatibility:** the precompiled Linux x86_64 `.so` plugins support MuJoCo **3.9.0–3.16.0**: every minor release from 3.9.0 through 3.16.0. MuJoCo 3.12.0 below is the recommended sample environment, not the plugin's compatibility limit.

### Python samples

- Python 3 compatible with the selected ROS2 distribution
- `mujoco == 3.12.0` (recommended for reproducing the sample commands)
- `numpy`
- ROS2 (`rclpy`, `sensor_msgs`; the samples publish tactile data to ROS2 by default, so source `/opt/ros/<distro>/setup.bash` before running them. These packages are provided by the ROS2 installation.)

Install:

```bash
python -m pip install mujoco==3.12.0 numpy
```

### C++ samples

- C++ 20
- CMake >= 3.22
- MuJoCo 3.12.0 (recommended; the bundled `.so` plugins support the versions listed above)
- GLFW3
- TinyXML2
- ROS2 (`rclcpp`, `sensor_msgs`; you need to `source /opt/ros/<distro>/setup.bash` before configuring CMake, otherwise `find_package(rclcpp REQUIRED)` fails outright at configure time)

On Ubuntu:

```bash
sudo apt install build-essential cmake libglfw3-dev libtinyxml2-dev
```

## Quick Start

The samples publish tactile data to ROS2 by default. Source the matching ROS2 environment before running them:

```bash
source /opt/ros/<distro>/setup.bash   # e.g. source /opt/ros/jazzy/setup.bash
```

### 1. Python: the fastest way to try Loong

Go to the repository root:

```bash
cd TipSim
```

Run the minimal tactile-array sample:

```bash
python samples/tip/python/loong96.py
```

### 2. Python: run the dexterous-hand grasp sample

```bash
python samples/inspire/python/rh56e2_hold_cylinder.py
```

This sample shows the RH56E2 dexterous hand's tactile surfaces responding in real time while it grasps.

### 3. Python: run the Universa samples

```bash
python samples/tip/python/universa_finger.py
```

Select the pressing object before starting with `--object sphere|cuboid|cube|nut` (default: `sphere`).
After building the C++ samples, you can use the same option with the C++ version. Both versions use the same scene and nut meshes:

```bash
python samples/tip/python/universa_finger.py --object nut
./build/universa_finger --object nut
```

Or run the full robot-arm pick-and-place scene:

```bash
python samples/tip/python/universa_pick_and_place.py
```

If the plugin fails to load, check its runtime dependencies first:

```bash
ldd -r lib/libtip_tactile_universa.so
```

See [Troubleshooting](#troubleshooting) for details.

## Build C++ Samples

Run the following commands from the repository root. The CMake project builds the six C++ sample programs; the tactile plugins under `lib/` are already precompiled.

### Configure and build

You need to source your ROS2 environment before configuring CMake (`find_package(rclcpp REQUIRED)` relies on the `CMAKE_PREFIX_PATH` it sets):

```bash
source /opt/ros/<distro>/setup.bash   # e.g. source /opt/ros/jazzy/setup.bash
```

If MuJoCo is already installed as a CMake package on a system path:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

If MuJoCo is installed in a custom location:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco

cmake --build build -j$(nproc)
```

Once built, you can run them directly (keep the ROS2 environment sourced in the same terminal):

```bash
./build/loong96
./build/rh56e2_hold_cylinder
./build/rh56e2_grasp_jelly
./build/universa_finger
./build/universa_pick_and_place
./build/universa_grasp_jelly
```

## Examples

All tactile sensors explicitly configure their array layout: `size` and `coords` for Loong, or `npoints` and `coords` for Universa. Each Universa sensor uses the default 80-taxel layout, with 10 rows × 8 columns at 2.5 mm pitch.

| Example | Plugin | Python | C++ | Scene |
| --- | --- | --- | --- | --- |
| Standalone tactile array | Loong | `samples/tip/python/loong96.py` | `build/loong96` | `samples/tip/scene/loong96.xml` |
| RH56E2 dexterous-hand grasp | Loong | `samples/inspire/python/rh56e2_hold_cylinder.py` | `build/rh56e2_hold_cylinder` | `samples/inspire/scene/rh56e2_hold_cylinder.xml` |
| RH56E2 jelly grasp | Loong | `samples/inspire/python/rh56e2_grasp_jelly.py` | `build/rh56e2_grasp_jelly` | `samples/inspire/scene/rh56e2_grasp_jelly.xml` |
| Single-finger continuous press | Universa | `samples/tip/python/universa_finger.py` | `build/universa_finger` | `samples/tip/scene/universa_finger.xml` |
| UR5e pick-and-place | Universa | `samples/tip/python/universa_pick_and_place.py` | `build/universa_pick_and_place` | `samples/tip/scene/universa_pick_and_place.xml` |
| Universa gripper jelly grasp | Universa | `samples/tip/python/universa_grasp_jelly.py` | `build/universa_grasp_jelly` | `samples/tip/scene/universa_grasp_jelly.xml` |

## Reproducible Runs (Random Seed)

All six sample scenes set a fixed `seed` for each tactile sensor. A fixed seed makes the sensor's random error sequence reproducible, but does not by itself guarantee byte-identical simulation output or ROS2 messages. Repeated comparisons also require the same MuJoCo version, runtime environment, model, initial state, control inputs, and sensor parameters. ROS2 publishing runs asynchronously and includes timestamps.

The current seed settings are listed below. Within a scene, sensors with fixed seeds use distinct values. Both RH56E2 scenes assign seeds `1`–`17` in XML sensor order, with the same seed assigned to each named sensor across the two scenes.

| Scene | Sensors | Seeds |
| --- | --- | --- |
| `tip/scene/loong96.xml` | 1 | `1` |
| `tip/scene/universa_finger.xml` | 1 | `1` |
| `tip/scene/universa_grasp_jelly.xml` | 2 | `1`–`2` |
| `tip/scene/universa_pick_and_place.xml` | 2 | `1`–`2` |
| `inspire/scene/rh56e2_hold_cylinder.xml` | 17 | `1`–`17` |
| `inspire/scene/rh56e2_grasp_jelly.xml` | 17 | `1`–`17` |

Change the seed to use a different random error sequence. Omit it to use a randomly initialized sequence on each run:

```xml
<config key="seed" value="42"/>   <!-- fixed sensor random seed -->
<!-- omit seed to use random initialization -->
```

See the root [README.md](../README.md#sensor-parameter-customization) for the full sensor parameter table and how the seed drives the noise model.

## ROS2 Tactile Publishing

All six samples publish tactile data to ROS2 by default, in both Python and C++. Each taxel's force is clipped to 0–50 N and linearly mapped to 0–65535 in a `sensor_msgs/Image` message with `mono16` encoding. This encoding range is separate from the sensor's `full_scale` parameter. The publishing logic lives in [common/tactile_ros2.py](common/tactile_ros2.py) (Python, `TactileRos2Publisher`) and [common/tactile_ros2.h](common/tactile_ros2.h) (C++, `tip::TactileRos2Publisher`); both use the same topic layout and force encoding.

ROS2 node creation, publishing, and callback processing run on a background thread. That thread reads the current `mjData.sensordata` values at a target rate of 30 Hz by default, rather than publishing a frame on every physics step.

| Sample | Topic |
| --- | --- |
| Standalone tactile array (loong96) | `/observation/right_hand/tactile_42` |
| Single-finger continuous press (universa_finger) | `/observation/right_hand/tactile_54` |
| UR5e pick-and-place (universa_pick_and_place) | `/observation/right_hand/tactile_54`, `/observation/left_hand/tactile_54` |
| RH56E2 dexterous-hand grasp (rh56e2_hold_cylinder) | 12 topics: `/observation/right_hand/tactile_{61,52,54,42,43,32,33,22,23,12,13}` and `/observation/right_hand/tactile_dmz_zz` |
| RH56E2 jelly grasp (rh56e2_grasp_jelly) | `/observation/right_hand/tactile_{43,42,54,52}` |
| Universa gripper jelly grasp (universa_grasp_jelly) | `/observation/right_hand/tactile_54`, `/observation/left_hand/tactile_54` |

The RH56E2 samples concatenate each published finger's zd (3×3) and zj (12×8) into **15×8** and publish on the
original zj topic, preserving the sensors' original row and column order. The top three rows contain
zd with two zero columns on the left and three on the right, followed by twelve rows of zj.
The five finger topic suffixes are `52` (thumb), `42` (index), `32` (middle), `22` (ring), and `12` (pinky).
The cylinder-grasp sample publishes all five fingers; the jelly-grasp sample publishes only the thumb and index finger. Separate zd topics are not published. Each merged image uses `mono16`, with `step=16` and 240 bytes of pixel data per frame.

After starting a sample, use the ROS2 tools to check publishing. Replace the topic below with one published by that sample:

```bash
ros2 topic list | grep tactile
ros2 topic hz /observation/right_hand/tactile_42
```

Each topic entry uses `(topic, sensor_name, rows, cols)`. Python accepts an optional fifth entry,
`(prepend_sensor_name, rows, cols)`, to prepend an additional surface; C++ uses the corresponding
`prepend_sensor_name`, `prepend_rows`, and `prepend_cols` fields.

## 3D Tactile Visualization (Octopus)

Octopus is a separately released desktop application for multimodal data acquisition and 3D tactile visualization. It subscribes to the ROS2 tactile topics above and is not built by TipSim's CMake project. The package [ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb](https://github.com/tiplab/TipSim/releases/download/octopus-host-1.1.2/ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb) targets **Ubuntu 24.04 (amd64) with ROS2 Jazzy**. Install ROS2 Jazzy first. This build also declares FFmpeg dependencies at version 8.1.1 or later, including `libavcodec62`, `libavformat62`, and `libavutil60`. APT can resolve these dependencies only if compatible versions are available from your configured package sources.

Download and install:

```bash
gh release download octopus-host-1.1.2 --repo tiplab/TipSim \
  --pattern 'ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb'
sudo apt install ./ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb
source /opt/ros/jazzy/setup.bash
```

Launch the 3D tactile view for the corresponding scene:

```bash
ros2 run octopus octopus --mode=inspire-rh56e2 --domain-id=0  # RH56E2 hand
ros2 run octopus octopus --mode=universa --domain-id=0  # Universa gripper
```

For the default multimodal data acquisition interface:

```bash
ros2 run octopus octopus --domain-id=0
```

`--domain-id` must match the ROS2 domain of the simulation publishing the tactile data. The commands above select domain 0, which is also the samples' default when `ROS_DOMAIN_ID` is unset. For another domain, set both applications explicitly. For example, use two terminals:

```bash
# Terminal 1: simulation (from the TipSim repository root)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=41
python samples/inspire/python/rh56e2_hold_cylinder.py

# Terminal 2: tactile visualization
source /opt/ros/jazzy/setup.bash
ros2 run octopus octopus --mode=inspire-rh56e2 --domain-id=41
```

`inspire-rh56e2` displays the hand's 11 standard tactile topics, including the merged zd/zj data on `52`, `42`, `32`, `22`, and `12`. The separate `tactile_dmz_zz` topic remains published by `rh56e2_hold_cylinder` but is not included in this view. `universa` displays the Universa gripper's `tactile_54` data; the panels for each side show the data published for that side. See [ROS2 Tactile Publishing](#ros2-tactile-publishing) for the topics emitted by each sample.

## Plugin Path Override

By default, the samples load the plugin from the repository's `lib/` directory.

The Loong C++ sample lets you override the plugin path with an environment variable:

```bash
TIP_LOONG_PLUGIN=/absolute/path/libtip_tactile_loong.so \
  ./build/loong96
```

If you're integrating TipSim into your own project, we recommend using an absolute path, an install path, or having your application manage the plugin directory itself.

## Troubleshooting

### `mj_loadPluginLibrary()` fails to load

First check the plugin's dependencies and unresolved symbols:

```bash
ldd -r lib/libtip_tactile_loong.so
ldd -r lib/libtip_tactile_universa.so
```

Look specifically for:

```text
not found
undefined symbol
```

### Can't find the MuJoCo shared library (for example, `libmujoco.so.3.12.0`)

Check the Python MuJoCo version:

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

The bundled `.so` plugins support MuJoCo 3.9.0–3.16.0. An error naming `libmujoco.so.3.12.0` means a component was linked against that library version. Make that library available at runtime, or rebuild the affected C++ program against the installed MuJoCo version.

If MuJoCo was only installed via a Python wheel, running `ldd` on its own may not be able to resolve the wheel's MuJoCo library from the system library path — trust the actual result of running the Python samples with a supported MuJoCo version instead.

If you're using a local C/C++ install, check:

```bash
ldconfig -p | grep mujoco
```

If MuJoCo is installed in a custom location, add its library directory to the current terminal:

```bash
export LD_LIBRARY_PATH=/path/to/mujoco/lib:$LD_LIBRARY_PATH
```

### CMake can't find MuJoCo

Point it at the MuJoCo CMake package explicitly:

```bash
cmake -S . -B build \
  -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco
```

### CMake can't find `rclcpp` / `sensor_msgs`

Ensure that ROS2 and the `rclcpp` / `sensor_msgs` packages are installed, then source the matching environment:

```bash
source /opt/ros/<distro>/setup.bash
```

then re-run `cmake -S . -B build ...`.

### Python sample fails with `ModuleNotFoundError: No module named 'rclpy'`

`rclpy` is provided by the ROS2 installation. Use a Python interpreter compatible with that installation and source the matching environment before running:

```bash
source /opt/ros/<distro>/setup.bash
```

### `undefined symbol: deflateReset`

This usually means the plugin's zlib dependency isn't resolving correctly in your environment. Check:

```bash
ldd -r lib/libtip_tactile_universa.so | grep -E "libz|undefined"
```

On Ubuntu, confirm the zlib runtime library is available:

```bash
sudo apt install zlib1g
```

If it still fails, make sure you're using the latest Universa plugin build from this repository, and aren't mixing an old `.so` with a new one.

### Checking whether a plugin still embeds a developer-machine absolute path

After publishing or moving a plugin, you can check:

```bash
readelf -d lib/libtip_tactile_universa.so | grep -E "RPATH|RUNPATH"
```

A properly released build should use a relative library search path, not depend on a specific developer machine's private `/home/...` or `/opt/...` paths.
