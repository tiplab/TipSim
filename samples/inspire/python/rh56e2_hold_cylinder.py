#!/usr/bin/env python3
"""RH56E2 循环抓握圆柱体，读取 17 个触觉表面并发布 12 路 ROS2 数据。"""
import os
import sys
import numpy as np
import mujoco
from mujoco import viewer as mjviewer

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "samples", "common"))
from tactile_ros2 import TactileRos2Publisher  # noqa: E402
_SO = os.environ.get("TIP_LOONG_PLUGIN") or os.path.join(_REPO_ROOT, "lib", "libtip_tactile_loong.so")
_MODEL = os.path.join(_REPO_ROOT, "samples", "inspire", "scene", "rh56e2_hold_cylinder.xml")

# 手指屈曲执行器（position servo）；屈曲方向为关节负角。
GRASP_ACTUATORS = [
    "sz_zj_actuator", "sz_zf_actuator", "zz_zj_actuator", "zz_zf_actuator",
    "wmz_zj_actuator", "wmz_zf_actuator", "xmz_zj_actuator", "xmz_zf_actuator",
    "dmz_jd_actuator",
]

# 触觉表面阵列尺寸，与场景 XML 保持一致。
SURFACE_SHAPES = {
    "zhangxin_touch_face": (8, 14),
    "sz_zj_touch_face": (12, 8), "zz_zj_touch_face": (12, 8),
    "wmz_zj_touch_face": (12, 8), "xmz_zj_touch_face": (12, 8), "dmz_zj_touch_face": (12, 8),
    "sz_zf_touch_face": (10, 8), "zz_zf_touch_face": (10, 8),
    "wmz_zf_touch_face": (10, 8), "xmz_zf_touch_face": (10, 8),
    "dmz_zf_touch_face": (12, 8), "dmz_zz_touch_face": (3, 3),
    "sz_zd_touch_face": (3, 3), "zz_zd_touch_face": (3, 3),
    "wmz_zd_touch_face": (3, 3), "xmz_zd_touch_face": (3, 3), "dmz_zd_touch_face": (3, 3),
}

SURF_LABEL = {
    "zhangxin_touch_face": "Palm",
    "dmz_zd_touch_face": "Thumb end",  "dmz_zj_touch_face": "Thumb tip",
    "dmz_zf_touch_face": "Thumb pad",  "dmz_zz_touch_face": "Thumb mid",
    "sz_zd_touch_face": "Index end",   "sz_zj_touch_face": "Index tip",   "sz_zf_touch_face": "Index pad",
    "zz_zd_touch_face": "Middle end",  "zz_zj_touch_face": "Middle tip",  "zz_zf_touch_face": "Middle pad",
    "wmz_zd_touch_face": "Ring end",   "wmz_zj_touch_face": "Ring tip",   "wmz_zf_touch_face": "Ring pad",
    "xmz_zd_touch_face": "Pinky end",  "xmz_zj_touch_face": "Pinky tip",  "xmz_zf_touch_face": "Pinky pad",
}
PALM = "zhangxin_touch_face"
MOSAIC = [
    ["dmz_zd_touch_face", "sz_zd_touch_face", "zz_zd_touch_face", "wmz_zd_touch_face", "xmz_zd_touch_face"],
    ["dmz_zj_touch_face", "sz_zj_touch_face", "zz_zj_touch_face", "wmz_zj_touch_face", "xmz_zj_touch_face"],
    ["dmz_zf_touch_face", "sz_zf_touch_face", "zz_zf_touch_face", "wmz_zf_touch_face", "xmz_zf_touch_face"],
    ["dmz_zz_touch_face", PALM, PALM, PALM, PALM],
]

# 触觉表面到 ROS2 话题后缀的映射。
TACTILE_TOPIC_ID = {
    "zhangxin_touch_face": "61",
    "dmz_zj_touch_face": "52", "dmz_zf_touch_face": "54",
    "sz_zj_touch_face": "42", "sz_zf_touch_face": "43",
    "zz_zj_touch_face": "32", "zz_zf_touch_face": "33",
    "wmz_zj_touch_face": "22", "wmz_zf_touch_face": "23",
    "xmz_zj_touch_face": "12", "xmz_zf_touch_face": "13",
}


def tactile_ros2_topics(surf):
    out = []
    for key, (_, _, rows, cols) in surf.items():
        if key.endswith("_zd_touch_face"):
            continue  # 指端拼接到对应 zj 话题, 不再单独发布
        suffix = TACTILE_TOPIC_ID.get(key, key[:-len("_touch_face")] if key.endswith("_touch_face") else key)
        entry = (f"/observation/right_hand/tactile_{suffix}", key + "_tactile", rows, cols)
        if key.endswith("_zj_touch_face"):
            zd_key = key.replace("_zj_touch_face", "_zd_touch_face")
            _, _, zd_rows, zd_cols = surf[zd_key]
            entry += ((zd_key + "_tactile", zd_rows, zd_cols),)
        out.append(entry)
    return out


STEPS = 3000       # 抓取/松开各跑多少个物理步
TARGET = -1.3         # 手指屈曲目标角（rad）

def surface_index(m):
    """返回触觉表面到 (adr, dim, rows, cols) 的映射。"""
    out = {}
    for i in range(m.nsensor):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i)
        if name is None:
            continue
        key = name[:-len("_tactile")] if name.endswith("_tactile") else name
        if key not in SURFACE_SHAPES:
            continue
        rows, cols = SURFACE_SHAPES[key]
        dim = m.sensor_dim[i]
        if dim == rows * cols:
            out[key] = (m.sensor_adr[i], dim, rows, cols)
    return out


def report(d, surf, title):
    """打印各表面的触觉统计并返回全手合力。"""
    print(f"\n===== {title} =====")
    grand = 0.0
    for key, (adr, dim, rows, cols) in surf.items():
        arr = d.sensordata[adr:adr + dim]
        total = float(arr.sum())
        grand += total
        print(f"  {SURF_LABEL[key]:12s} [{rows}x{cols:>2d}]  Σ={total:7.3f}N  "
              f"peak={float(arr.max()):6.3f}N  active={int((arr > 1e-6).sum()):3d}")
    print(f"  ----- 全手合力 Σ = {grand:.3f} N -----")
    return grand


def main():
    # 插件必须先于模型加载。
    mujoco.mj_loadPluginLibrary(_SO)

    m = mujoco.MjModel.from_xml_path(_MODEL)
    d = mujoco.MjData(m)

    # 关节下限对应拇指张开姿态。
    dmz_jd_jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "dmz_jd_joint")
    d.qpos[m.jnt_qposadr[dmz_jd_jid]] = m.jnt_range[dmz_jd_jid][0]

    mujoco.mj_forward(m, d)

    surf = surface_index(m)
    act_ids = {a: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in GRASP_ACTUATORS}
    ros2_pub = TactileRos2Publisher(m, d, tactile_ros2_topics(surf))

    viewer = mjviewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)

    def run_phase(target_start, target_end, phase_name):
        for step in range(STEPS):
            frac = min(1.0, step / (0.6 * STEPS))
            angle = target_start + (target_end - target_start) * frac
            for i in act_ids.values():
                d.ctrl[i] = angle
            mujoco.mj_step(m, d)
            if not viewer.is_running():
                return False
            viewer.sync()
        return True

    # 循环抓握与松开，关闭窗口后退出。
    report(d, surf, "初始（未接触）")
    cycle = 0
    while viewer.is_running():
        cycle += 1
        if not run_phase(0.0, TARGET, f"grasping (cycle {cycle})"):
            break
        grasp_force = report(d, surf, f"抓取中 (cycle {cycle})")
        print("[结论]", "抓取时检测到触觉合力，读数正常。" if grasp_force > 0.1 else
              "抓取时未检测到接触力——可能需调 cylinder 位置或屈曲目标角。")
        if not run_phase(TARGET, 0.0, f"releasing (cycle {cycle})"):
            break
        report(d, surf, f"松手后 (cycle {cycle})")

    ros2_pub.stop()
    viewer.close()


if __name__ == "__main__":
    main()
