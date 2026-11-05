#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Universa 单指按压示例，支持球体、长方体、正方体和螺母。

用法: python universa_finger.py --object sphere"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "samples", "common"))

SCENE = os.path.join(HERE, "..", "scene", "universa_finger.xml")
PLUGIN = os.path.join(REPO_ROOT, "lib", "libtip_tactile_universa.so")
ROWS, COLS, PITCH_MM = 10, 8, 2.5
DRAW_EVERY = 15
PROBE_TYPES = ("sphere", "cuboid", "cube", "nut")

# 沿用目标系统的 tactile_54 通道命名。
ROS2_TOPIC = "/observation/right_hand/tactile_54"

# 施加力线性增加至上限后保持；传感器量程由 XML 配置。
F_MAX = 35.0
RAMP_TIME = 8.0


def applied_force(t):
    """返回 t 秒时沿 -Z 方向施加的力（N），达到上限后保持。"""
    return min(F_MAX, F_MAX * t / RAMP_TIME)


def load_model(object_type="sphere"):
    """编译前选择压头，避免未选物块参与碰撞。"""
    if object_type not in PROBE_TYPES:
        raise ValueError(f"未知按压对象: {object_type}; 可选 {', '.join(PROBE_TYPES)}")
    mujoco.mj_loadPluginLibrary(PLUGIN)
    root = ET.parse(SCENE).getroot()
    compiler = root.find("compiler")
    meshdir = os.path.join(os.path.dirname(SCENE), compiler.get("meshdir"))
    compiler.set("meshdir", os.path.abspath(meshdir))
    probe = root.find("worldbody/body[@name='probe']")
    geom = probe.find("geom")
    if object_type == "nut":
        probe.remove(geom)
        # Separate convex pieces preserve the hole; a single hull would fill it.
        for mesh in root.findall("asset/mesh"):
            name = mesh.get("name")
            if name.startswith("probe_nut_hull"):
                ET.SubElement(probe, "geom", name=name, mesh=name, **{"class": "probe_nut"})
    else:
        geom.set("class", f"probe_{object_type}")
    root.find("custom/text[@name='universa_rigid']").set(
        "data", " ".join(g.get("name") for g in probe.findall("geom")))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def main(object_type="sphere"):
    from tactile_ros2 import TactileRos2Publisher

    m = load_model(object_type)
    d = mujoco.MjData(m)
    print(f"按压对象: {object_type}")
    adr = m.sensor("finger_tactile").adr[0]  # 获取触觉传感器在 mjData.sensordata 中的起始地址
    probe_dof = m.jnt_dofadr[m.joint("probe_z").id]

    viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)

    ros2_pub = TactileRos2Publisher(m, d, [(ROS2_TOPIC, "finger_tactile", ROWS, COLS)])

    k = 0
    while viewer.is_running():
        f = applied_force(d.time)
        d.qfrc_applied[probe_dof] = -f  # 沿 -Z 压向垫面
        mujoco.mj_step(m, d)
        k += 1
        if k % DRAW_EVERY == 0:
            viewer.sync()
            g = d.sensordata[adr:adr + ROWS * COLS].reshape(ROWS, COLS) # 触觉数据读取
            sensed = float(g.sum())
            print("t=%6.2fs  applied=%6.2fN  sensed=%6.2fN" % (d.time, f, sensed))

    ros2_pub.stop()
    viewer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--object", dest="object_type", choices=PROBE_TYPES, default="sphere",
                        help="按压对象: sphere 球体(默认), cuboid 长方体, cube 正方体, nut 螺母")
    args = parser.parse_args()
    main(args.object_type)
