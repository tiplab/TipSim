[English](README.md) | **简体中文**

# TipSim Samples

本文介绍本目录中的 Python / C++ 示例、构建方式、ROS2 发布、3D 触觉可视化和常见问题排查。如何在自己的项目中加载传感器插件、声明传感器并读取 `sensordata`，见根目录 [README.zh-CN.md](../README.zh-CN.md)。

## Requirements

除插件的运行依赖外，运行或构建示例还需要以下依赖：

> **插件兼容性：** 预编译 Linux x86_64 `.so` 插件支持 MuJoCo **3.9.0–3.16.0**，即 3.9.0、3.10.0、3.11.0、3.12.0、3.13.0、3.14.0、3.15.0 和 3.16.0。下文使用 MuJoCo 3.12.0 作为示例推荐环境。

### Python 示例

- 与所选 ROS2 发行版兼容的 Python 3
- `mujoco == 3.12.0`（推荐用于复现文档中的示例命令）
- `numpy`
- ROS2（`rclpy`、`sensor_msgs`；示例默认向 ROS2 发布触觉数据，运行前需执行 `source /opt/ros/<distro>/setup.bash`。这两个包由 ROS2 安装提供。）

安装：

```bash
python -m pip install mujoco==3.12.0 numpy
```

### C++ 示例

- C++ 20
- CMake ≥ 3.22
- MuJoCo 3.12.0（推荐；内置 `.so` 插件支持上文列出的版本）
- GLFW3
- TinyXML2
- ROS2（`rclcpp`、`sensor_msgs`；配置 CMake 前需要先 `source /opt/ros/<distro>/setup.bash`，否则 `find_package(rclcpp REQUIRED)` 会在配置阶段直接报错退出）

Ubuntu 示例：

```bash
sudo apt install build-essential cmake libglfw3-dev libtinyxml2-dev
```

## Quick Start

示例默认向 ROS2 发布触觉数据，运行前需加载对应发行版的 ROS2 环境：

```bash
source /opt/ros/<distro>/setup.bash   # 例如 source /opt/ros/jazzy/setup.bash
```

### 1. Python：最快体验 Loong

进入仓库根目录：

```bash
cd TipSim
```

运行最小触觉阵列示例：

```bash
python samples/tip/python/loong96.py
```

### 2. Python：运行灵巧手抓取示例

```bash
python samples/inspire/python/rh56e2_hold_cylinder.py
```

该示例展示 RH56E2 灵巧手多个触觉表面在抓取过程中的实时输出。

### 3. Python：运行 Universa 示例

```bash
python samples/tip/python/universa_finger.py
```

运行前可用 `--object sphere|cuboid|cube|nut` 选择球体、长方体、正方体或螺母，默认是球体。
完成 C++ 示例构建后，也可使用相同参数。Python 和 C++ 版本使用同一份场景和螺母网格：

```bash
python samples/tip/python/universa_finger.py --object nut
./build/universa_finger --object nut
```

或运行完整的机械臂取放场景：

```bash
python samples/tip/python/universa_pick_and_place.py
```

## Build C++ Samples

以下命令均在仓库根目录执行。CMake 工程编译六个 C++ 示例程序；`lib/` 下的触觉插件已预编译完成。

### 配置并编译

CMake 配置前需要先 source ROS2 环境（`find_package(rclcpp REQUIRED)` 依赖它设置的 `CMAKE_PREFIX_PATH`）：

```bash
source /opt/ros/<distro>/setup.bash   # 例如 source /opt/ros/jazzy/setup.bash
```

如果 MuJoCo 已安装到系统路径，且提供 CMake 包配置文件：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

如果 MuJoCo 安装在自定义目录：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco

cmake --build build -j$(nproc)
```

编译完成后，可在已加载 ROS2 环境的终端中运行：

```bash
./build/loong96
./build/rh56e2_hold_cylinder
./build/rh56e2_grasp_jelly
./build/universa_finger
./build/universa_pick_and_place
./build/universa_grasp_jelly
```

## Examples

所有触觉传感器均显式配置阵列布局：Loong 设置 `size` 和 `coords`，Universa 设置 `npoints` 和 `coords`。每个 Universa 传感器均使用默认的 80 点布局，按 10 行 × 8 列排列，间距为 2.5 mm。

| Example | Plugin | Python | C++ | Scene |
| --- | --- | --- | --- | --- |
| 独立触觉阵列 | Loong | `samples/tip/python/loong96.py` | `build/loong96` | `samples/tip/scene/loong96.xml` |
| RH56E2 灵巧手抓取 | Loong | `samples/inspire/python/rh56e2_hold_cylinder.py` | `build/rh56e2_hold_cylinder` | `samples/inspire/scene/rh56e2_hold_cylinder.xml` |
| RH56E2 灵巧手果冻抓取 | Loong | `samples/inspire/python/rh56e2_grasp_jelly.py` | `build/rh56e2_grasp_jelly` | `samples/inspire/scene/rh56e2_grasp_jelly.xml` |
| 单指连续按压 | Universa | `samples/tip/python/universa_finger.py` | `build/universa_finger` | `samples/tip/scene/universa_finger.xml` |
| UR5e 取放 | Universa | `samples/tip/python/universa_pick_and_place.py` | `build/universa_pick_and_place` | `samples/tip/scene/universa_pick_and_place.xml` |
| Universa 夹爪果冻抓取 | Universa | `samples/tip/python/universa_grasp_jelly.py` | `build/universa_grasp_jelly` | `samples/tip/scene/universa_grasp_jelly.xml` |

## 可复现运行（随机种子）

六个示例场景均已为每个触觉传感器配置固定 `seed`。固定种子可复现传感器的随机误差序列，但仅固定种子不足以保证仿真输出或 ROS2 消息逐位一致。重复比较时，还需保持 MuJoCo 版本、运行环境、模型、初始状态、控制输入和传感器参数一致。ROS2 发布是异步进行的，消息中也包含时间戳。

各场景当前的种子配置如下。同一场景内，配置了固定种子的传感器使用不同数值。两个 RH56E2 场景均按 XML 中的传感器顺序分配 `1`–`17`，同名传感器在两个场景中使用相同种子。

| 场景 | 传感器数 | 种子 |
| --- | --- | --- |
| `tip/scene/loong96.xml` | 1 | `1` |
| `tip/scene/universa_finger.xml` | 1 | `1` |
| `tip/scene/universa_grasp_jelly.xml` | 2 | `1`–`2` |
| `tip/scene/universa_pick_and_place.xml` | 2 | `1`–`2` |
| `inspire/scene/rh56e2_hold_cylinder.xml` | 17 | `1`–`17` |
| `inspire/scene/rh56e2_grasp_jelly.xml` | 17 | `1`–`17` |

修改种子可使用另一组随机误差序列；省略该配置后，每次运行都会随机初始化：

```xml
<config key="seed" value="42"/>   <!-- 固定传感器随机种子 -->
<!-- 省略 seed 时使用随机初始化 -->
```

完整的传感器参数表及种子对随机误差的作用，见根目录 [README.zh-CN.md](../README.zh-CN.md#传感器参数自定义)。

## ROS2 触觉发布

六个示例的 Python / C++ 版本均默认向 ROS2 发布触觉数据：每个触点的力值先限制在 0–50 N，再线性映射到 0–65535，以 `sensor_msgs/Image` 消息、`mono16` 编码发布。该编码范围与传感器的 `full_scale` 参数相互独立。发布逻辑封装在 [common/tactile_ros2.py](common/tactile_ros2.py)（Python，`TactileRos2Publisher`）和 [common/tactile_ros2.h](common/tactile_ros2.h)（C++，`tip::TactileRos2Publisher`）中，两者使用相同的话题布局和力值编码。

ROS2 节点创建、消息发布和回调处理均在后台线程中完成。该线程以默认 30 Hz 的目标频率读取 `mjData.sensordata` 当前值，无需在每个物理步中发布一帧。

| 示例 | Topic |
| --- | --- |
| 独立触觉阵列（loong96） | `/observation/right_hand/tactile_42` |
| 单指连续按压（universa_finger） | `/observation/right_hand/tactile_54` |
| UR5e 取放（universa_pick_and_place） | `/observation/right_hand/tactile_54`、`/observation/left_hand/tactile_54` |
| RH56E2 灵巧手抓取（rh56e2_hold_cylinder） | 12 路：`/observation/right_hand/tactile_{61,52,54,42,43,32,33,22,23,12,13}`，以及 `/observation/right_hand/tactile_dmz_zz` |
| RH56E2 果冻抓取（rh56e2_grasp_jelly） | `/observation/right_hand/tactile_{43,42,54,52}` |
| Universa 夹爪果冻抓取（universa_grasp_jelly） | `/observation/right_hand/tactile_54`、`/observation/left_hand/tactile_54` |

RH56E2 示例将参与发布的手指的 zd（3×3）与 zj（12×8）拼接为 **15×8**，发送到原 zj 话题，
保持传感器原有行列顺序。顶部三行是 zd（左侧补 2 列零，右侧补 3 列零），下面十二行是 zj。
五指对应的话题后缀为 `52`（拇指）、`42`（食指）、`32`（中指）、`22`（无名指）、`12`（小指）。
圆柱体抓取示例发布五指数据，果冻抓取示例仅发布拇指和食指数据。zd 不单独发布。每幅合并图像采用 `mono16` 编码，`step=16`，每帧像素数据为 240 字节。

启动示例后，可用 ROS2 工具检查数据发布情况。请将下方话题替换为当前示例实际发布的话题：

```bash
ros2 topic list | grep tactile
ros2 topic hz /observation/right_hand/tactile_42
```

接入自己的项目时，每路 topic 使用 `(topic, sensor_name, rows, cols)`。Python 可在第五项传入
`(prepend_sensor_name, rows, cols)` 来指定拼接在顶部的额外表面；C++ 使用对应的
`prepend_sensor_name`、`prepend_rows`、`prepend_cols` 字段。

## 3D 触觉可视化 (Octopus)

Octopus 是独立发布的具身多模态数据采集和 3D 触觉可视化上位机应用，订阅上面的 ROS2 触觉话题，不参与 TipSim 的 CMake 构建。软件包 [ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb](https://github.com/tiplab/TipSim/releases/download/octopus-host-1.1.2/ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb) 面向 **Ubuntu 24.04（amd64）和 ROS2 Jazzy**。请先安装 ROS2 Jazzy。该构建还声明了 FFmpeg 8.1.1 或更高版本的依赖，包括 `libavcodec62`、`libavformat62` 和 `libavutil60`；只有当前配置的 APT 软件源提供兼容版本时，APT 才能解析并安装这些依赖。

下载并安装：

```bash
gh release download octopus-host-1.1.2 --repo tiplab/TipSim \
  --pattern 'ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb'
sudo apt install ./ros-jazzy-octopus_1.1.2-0noble.20261103.1_amd64.deb
source /opt/ros/jazzy/setup.bash
```

按场景启动 3D 触觉画面：

```bash
ros2 run octopus octopus --mode=inspire-rh56e2 --domain-id=0  # RH56E2 灵巧手
ros2 run octopus octopus --mode=universa --domain-id=0  # Universa 夹爪
```

启动默认的多模态数据采集界面：

```bash
ros2 run octopus octopus --domain-id=0
```

`--domain-id` 必须与发布触觉数据的仿真进程处于同一个 ROS2 domain。上述命令指定 domain 0，示例未设置 `ROS_DOMAIN_ID` 时也默认使用 0。使用其他 domain 时，两端都要显式设置，例如打开两个终端：

```bash
# 终端 1：仿真（在 TipSim 仓库根目录运行）
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=41
python samples/inspire/python/rh56e2_hold_cylinder.py

# 终端 2：触觉可视化
source /opt/ros/jazzy/setup.bash
ros2 run octopus octopus --mode=inspire-rh56e2 --domain-id=41
```

`inspire-rh56e2` 显示灵巧手的 11 路标准触觉话题，其中 `52`、`42`、`32`、`22`、`12` 包含合成后的 zd/zj 数据。`rh56e2_hold_cylinder` 仍发布独立的 `tactile_dmz_zz`，该视图未包含这一路。`universa` 显示 Universa 夹爪的 `tactile_54` 数据，各侧面板显示对应侧发布的数据。每个示例实际发布的话题见 [ROS2 触觉发布](#ros2-触觉发布)。

## Plugin Path Override

示例默认从仓库的 `lib/` 目录加载插件。

Loong C++ 示例可通过环境变量覆盖插件路径：

```bash
TIP_LOONG_PLUGIN=/absolute/path/libtip_tactile_loong.so \
  ./build/loong96
```

如果你将 TipSim 集成到自己的项目，建议使用绝对路径、安装路径或由应用程序统一管理插件目录。

## Troubleshooting

### `mj_loadPluginLibrary()` 加载失败

先检查插件依赖及未解析符号：

```bash
ldd -r lib/libtip_tactile_loong.so
ldd -r lib/libtip_tactile_universa.so
```

重点检查是否存在：

```text
not found
undefined symbol
```

### 找不到 MuJoCo 共享库（例如 `libmujoco.so.3.12.0`）

确认 Python MuJoCo 版本：

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

仓库内置的 `.so` 插件支持 MuJoCo 3.9.0–3.16.0。报错中出现 `libmujoco.so.3.12.0`，说明某个组件链接到了该版本的库。需要让运行时能找到该库，或使用已安装的 MuJoCo 版本重新编译相关 C++ 程序。

如果只通过 Python wheel 安装 MuJoCo，单独执行 `ldd` 时可能无法从系统库路径解析 wheel 内的 MuJoCo 库；Python 示例应以使用受支持 MuJoCo 版本的实际运行结果为准。

如果使用本地 C/C++ 安装，可检查：

```bash
ldconfig -p | grep mujoco
```

MuJoCo 安装在自定义目录时，可将其运行库目录加入当前终端：

```bash
export LD_LIBRARY_PATH=/path/to/mujoco/lib:$LD_LIBRARY_PATH
```

### CMake 找不到 MuJoCo

显式指定 MuJoCo 的 CMake 包配置文件所在目录：

```bash
cmake -S . -B build \
  -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco
```

### CMake 找不到 `rclcpp` / `sensor_msgs`

确认 ROS2 及 `rclcpp` / `sensor_msgs` 包已安装，再加载对应环境：

```bash
source /opt/ros/<distro>/setup.bash
```

再重新运行 `cmake -S . -B build ...`。

### Python 运行示例时 `ModuleNotFoundError: No module named 'rclpy'`

`rclpy` 由 ROS2 安装提供。请使用与该安装兼容的 Python 解释器，并在运行前加载对应环境：

```bash
source /opt/ros/<distro>/setup.bash
```

### `undefined symbol: deflateReset`

该错误通常表示插件或其运行环境中的 zlib 依赖未正确解析。先检查：

```bash
ldd -r lib/libtip_tactile_universa.so | grep -E "libz|undefined"
```

在 Ubuntu 上确认 zlib 运行库可用：

```bash
sudo apt install zlib1g
```

如果仍然报错，请确认使用的是仓库最新提供的 Universa 预编译插件，不要混用旧版本 `.so`。

### 检查插件是否仍包含开发机绝对路径

发布或移动插件后可检查：

```bash
readelf -d lib/libtip_tactile_universa.so | grep -E "RPATH|RUNPATH"
```

正式发布版本应使用相对运行库搜索路径，不应依赖某台开发机的 `/home/...` 或 `/opt/...` 私有路径。
