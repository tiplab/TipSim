#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Loong 96 点触觉阵列落块示例。"""
import os
import sys
import numpy as np
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "samples", "common"))
from tactile_ros2 import TactileRos2Publisher  # noqa: E402

SCENE = os.path.join(HERE, "..", "scene", "loong96.xml")
PLUGIN = os.path.join(REPO_ROOT, "lib", "libtip_tactile_loong.so")
ROWS, COLS = 12, 8
ROW_PITCH_MM, COL_PITCH_MM = 1.17, 1.2297  # 触点间距与 XML 一致
DRAW_EVERY = 15

# 沿用目标系统的 tactile_42 通道命名。
ROS2_TOPIC = "/observation/right_hand/tactile_42"


def main():
    mujoco.mj_loadPluginLibrary(PLUGIN)
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    adr = m.sensor("loong96_tactile").adr[0] # 获取触觉传感器在 mjData.sensordata 中的起始地址

    viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)

    ros2_pub = TactileRos2Publisher(m, d, [(ROS2_TOPIC, "loong96_tactile", ROWS, COLS)])

    k = 0
    while viewer.is_running():
        mujoco.mj_step(m, d)
        k += 1
        if k % DRAW_EVERY == 0:
            viewer.sync()
            g = d.sensordata[adr:adr + ROWS * COLS].reshape(ROWS, COLS) # 触觉阵列插件的传感器数据, 直接从 mjData.sensordata 里读, 不需要任何专用的读数据扩展模块
            print("t=%.2fs  total=%.2fN  active=%d" % (d.time, g.sum(), int((g > 1e-6).sum())))

    ros2_pub.stop()
    viewer.close()


if __name__ == "__main__":
    main()
