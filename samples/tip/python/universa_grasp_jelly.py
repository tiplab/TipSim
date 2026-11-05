#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Universa 夹爪循环抓取、提起并放回柔性果冻，发布双侧触觉数据。"""
import os
import sys
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "samples", "common"))
from tactile_ros2 import TactileRos2Publisher  # noqa: E402

SCENE = os.path.join(HERE, "..", "scene", "universa_grasp_jelly.xml")
PLUGIN = os.path.join(REPO_ROOT, "lib", "libtip_tactile_universa.so")
ROWS, COLS = 10, 8  # 默认 10×8 触点阵列

# 右、左指分别对应 right_hand / left_hand 的 tactile_54。
ROS2_TOPICS = [
    ("/observation/right_hand/tactile_54", "right_finger_tactile", ROWS, COLS),
    ("/observation/left_hand/tactile_54", "left_finger_tactile", ROWS, COLS),
]
SETTLE_STEPS = 1000
GRIP_MAX = 210.0
GRIP_STEP = 0.15
GRASP_FORCE_THRESHOLD = 0.1
LIFT_HEIGHT = 0.08
LIFT_STEP = 0.0002
HOLD_STEPS = 300
DRAW_EVERY = 15


class Display:
    """MuJoCo 3D 窗口。"""

    def __init__(self, m, d):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)

    def update(self):
        self.viewer.sync()

    def alive(self):
        return self.viewer.is_running()

    def close(self):
        self.viewer.close()


def main():
    mujoco.mj_loadPluginLibrary(PLUGIN)
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)

    act      = m.actuator("fingers_actuator").id
    lift_act = m.actuator("lift_actuator").id
    adr_l    = m.sensor("left_finger_tactile").adr[0]
    adr_r    = m.sensor("right_finger_tactile").adr[0]

    ros2_pub = TactileRos2Publisher(m, d, ROS2_TOPICS)
    disp = Display(m, d)
    try:
        step_count = 0

        def tactile():
            gl = d.sensordata[adr_l:adr_l + ROWS * COLS].reshape(ROWS, COLS)
            gr = d.sensordata[adr_r:adr_r + ROWS * COLS].reshape(ROWS, COLS)
            return gl, gr

        def advance(phase):
            nonlocal step_count
            mujoco.mj_step(m, d)
            step_count += 1
            if step_count % DRAW_EVERY == 0:
                disp.update()
            return disp.alive()

        print("豆腐块在支架上稳定……")
        d.ctrl[act] = 0.0
        d.ctrl[lift_act] = 0.0
        for _ in range(SETTLE_STEPS):
            if not advance("settling jelly on stand"):
                return

        print("循环挤压/提起/放下/松开豆腐块……(关掉 3D 窗口结束)")
        cycle = 0
        grip = 0.0
        lift = 0.0
        while disp.alive():
            cycle += 1
            # 双侧合力均达阈值或闭合到行程上限后停止。
            while grip < GRIP_MAX:
                grip = min(grip + GRIP_STEP, GRIP_MAX)
                d.ctrl[act] = grip
                if not advance(f"SQUEEZE (cycle {cycle})"):
                    return
                gl, gr = tactile()
                if gl.sum() >= GRASP_FORCE_THRESHOLD and gr.sum() >= GRASP_FORCE_THRESHOLD:
                    break
            gl, gr = tactile()
            reason = "达到力阈值" if grip < GRIP_MAX else "到达 GRIP_MAX 兜底"
            print(f"[cycle {cycle}] 挤压到 grip={grip:.0f} ({reason})  left_total={gl.sum():.2f}N  right_total={gr.sum():.2f}N")
            for _ in range(HOLD_STEPS):
                if not advance(f"HOLD (cycle {cycle})"):
                    return
            # 提起。
            while lift < LIFT_HEIGHT:
                lift = min(lift + LIFT_STEP, LIFT_HEIGHT)
                d.ctrl[lift_act] = lift
                if not advance(f"LIFT (cycle {cycle})"):
                    return
            gl, gr = tactile()
            print(f"[cycle {cycle}] 提起到 {lift*1000:.0f}mm  left_total={gl.sum():.2f}N  right_total={gr.sum():.2f}N")
            for _ in range(HOLD_STEPS):
                if not advance(f"HOLD UP (cycle {cycle})"):
                    return
            # 放回支架。
            while lift > 0.0:
                lift = max(lift - LIFT_STEP, 0.0)
                d.ctrl[lift_act] = lift
                if not advance(f"LOWER (cycle {cycle})"):
                    return
            for _ in range(HOLD_STEPS):
                if not advance(f"HOLD DOWN (cycle {cycle})"):
                    return
            # 松开。
            while grip > 0.0:
                grip = max(grip - GRIP_STEP, 0.0)
                d.ctrl[act] = grip
                if not advance(f"RELEASE (cycle {cycle})"):
                    return
            for _ in range(HOLD_STEPS):
                if not advance(f"OPEN (cycle {cycle})"):
                    return
    finally:
        disp.close()
        ros2_pub.stop()


if __name__ == "__main__":
    main()
