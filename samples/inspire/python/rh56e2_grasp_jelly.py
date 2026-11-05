#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RH56E2 食指与拇指独立控制，循环捏起并放回柔性果冻。"""
import os
import sys
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "samples", "common"))
SCENE = os.path.join(HERE, "..", "scene", "rh56e2_grasp_jelly.xml")
PLUGIN = os.path.join(REPO_ROOT, "lib", "libtip_tactile_loong.so")

# 食指、拇指的指腹（zf）、指尖（zj）和指端（zd）阵列尺寸。
SURFACES = {
    "sz_zf_touch_face": (10, 8), "sz_zj_touch_face": (12, 8), "sz_zd_touch_face": (3, 3),
    "dmz_zf_touch_face": (12, 8), "dmz_zj_touch_face": (12, 8), "dmz_zd_touch_face": (3, 3),
}
SURF_LABEL = {
    "sz_zf_touch_face": "Index pad", "sz_zj_touch_face": "Index tip", "sz_zd_touch_face": "Index tip-end",
    "dmz_zf_touch_face": "Thumb pad", "dmz_zj_touch_face": "Thumb tip", "dmz_zd_touch_face": "Thumb tip-end",
}

# zd 居中补零后与 zj 合并为 15×8，保留原有行列顺序。
ROS2_TOPICS = [
    ("/observation/right_hand/tactile_43", "sz_zf_touch_face_tactile", 10, 8),
    ("/observation/right_hand/tactile_42", "sz_zj_touch_face_tactile", 12, 8,
     ("sz_zd_touch_face_tactile", 3, 3)),
    ("/observation/right_hand/tactile_54", "dmz_zf_touch_face_tactile", 12, 8),
    ("/observation/right_hand/tactile_52", "dmz_zj_touch_face_tactile", 12, 8,
     ("dmz_zd_touch_face_tactile", 3, 3)),
]

SETTLE_STEPS = 1000   # 张开状态下的稳定步数
DMZ_JD = -0.0          # 拇指根部闭合目标角（rad）
SZ_MAX = 1.35          # 食指屈曲上限（rad）
SZ_STEP = 0.0003       # 食指每步屈曲量（rad）
GRASP_FORCE_THRESHOLD = 1.0   # 单指三块表面的合力阈值（N）
LIFT_HEIGHT = 0.05     # 提升高度（m）
LIFT_STEP = 0.000015    # 每步升降量（m）
HOLD_STEPS = 300       # 各阶段保持步数
DRAW_EVERY = 15        # 显示刷新间隔（物理步）


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


def main(display_factory=Display, publish_ros2=True):
    """运行抓取循环，可替换显示器或关闭 ROS2 发布。"""
    mujoco.mj_loadPluginLibrary(PLUGIN)
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)

    sz_zf_act = m.actuator("sz_zf_actuator").id
    sz_zj_act = m.actuator("sz_zj_actuator").id
    dmz_jd_act = m.actuator("dmz_jd_actuator").id
    dmz_jd_joint = m.joint("dmz_jd_joint").id
    lift_act = m.actuator("lift_actuator").id

    adr = {key: m.sensor(key + "_tactile").adr[0] for key in SURFACES}

    # 关节下限对应拇指张开姿态。
    dmz_jd_open = float(m.jnt_range[dmz_jd_joint][0])
    d.qpos[m.jnt_qposadr[dmz_jd_joint]] = dmz_jd_open
    d.ctrl[dmz_jd_act] = dmz_jd_open

    ros2_pub = None
    disp = None
    try:
        if publish_ros2:
            from tactile_ros2 import TactileRos2Publisher
            ros2_pub = TactileRos2Publisher(m, d, ROS2_TOPICS)
        disp = display_factory(m, d)
        step_count = 0

        def tactile():
            return {key: d.sensordata[a:a + rows * cols].reshape(rows, cols)
                    for key, (rows, cols) in SURFACES.items()
                    for a in (adr[key],)}

        def advance(phase):
            nonlocal step_count
            mujoco.mj_step(m, d)
            step_count += 1
            if step_count % DRAW_EVERY == 0:
                disp.update()
            return disp.alive()

        print("果冻在支架上稳定……")
        d.ctrl[sz_zf_act] = 0.0
        d.ctrl[sz_zj_act] = 0.0
        d.ctrl[lift_act] = 0.0
        for _ in range(SETTLE_STEPS):
            if not advance("settling jelly on stand"):
                return

        print("循环捏紧/提起/放下/松开果冻……(关掉显示窗口结束)")
        cycle = 0
        sz_a = 0.0
        dmz_jd_a = dmz_jd_open
        # 按食指行程估算拇指步长，两指仍独立控制。
        dmz_jd_step = abs(DMZ_JD - dmz_jd_open) / (SZ_MAX / SZ_STEP)
        lift = 0.0
        while disp.alive():
            cycle += 1
            # 两指独立闭合，各自达到力阈值或行程上限后停止。
            sz_done = False
            dmz_done = False
            while not (sz_done and dmz_done):
                if not sz_done:
                    sz_a = min(sz_a + SZ_STEP, SZ_MAX)
                    d.ctrl[sz_zf_act] = -sz_a
                    d.ctrl[sz_zj_act] = -sz_a
                if not dmz_done:
                    dmz_jd_a = (min(dmz_jd_a + dmz_jd_step, DMZ_JD) if DMZ_JD > dmz_jd_a
                                else max(dmz_jd_a - dmz_jd_step, DMZ_JD))
                    d.ctrl[dmz_jd_act] = dmz_jd_a
                if not advance(f"SQUEEZE (cycle {cycle})"):
                    return
                grids_now = tactile()
                index_now = sum(float(grids_now[k].sum()) for k in ("sz_zf_touch_face", "sz_zj_touch_face", "sz_zd_touch_face"))
                thumb_now = sum(float(grids_now[k].sum()) for k in ("dmz_zf_touch_face", "dmz_zj_touch_face", "dmz_zd_touch_face"))
                if not sz_done and (index_now >= GRASP_FORCE_THRESHOLD or sz_a >= SZ_MAX):
                    sz_done = True
                if not dmz_done and (thumb_now >= GRASP_FORCE_THRESHOLD or dmz_jd_a == DMZ_JD):
                    dmz_done = True
            grids = tactile()
            sz_reason = "达到力阈值" if sz_a < SZ_MAX else "到达 SZ_MAX 兜底"
            dmz_reason = "达到力阈值" if dmz_jd_a != DMZ_JD else "到达 DMZ_JD 兜底"
            index_n = sum(float(grids[k].sum()) for k in ("sz_zf_touch_face", "sz_zj_touch_face", "sz_zd_touch_face"))
            thumb_n = sum(float(grids[k].sum()) for k in ("dmz_zf_touch_face", "dmz_zj_touch_face", "dmz_zd_touch_face"))
            print(f"[cycle {cycle}] 食指屈曲到 {sz_a:.2f}rad({sz_reason}) index={index_n:.2f}N  "
                  f"拇指合拢到 {dmz_jd_a:.2f}rad({dmz_reason}) thumb={thumb_n:.2f}N")
            for _ in range(HOLD_STEPS):
                if not advance(f"HOLD (cycle {cycle})"):
                    return
            # 提起。
            while lift < LIFT_HEIGHT:
                lift = min(lift + LIFT_STEP, LIFT_HEIGHT)
                d.ctrl[lift_act] = lift
                if not advance(f"LIFT (cycle {cycle})"):
                    return
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
            # 两指独立松开，回到初始张开姿态。
            while sz_a > 0.0 or dmz_jd_a != dmz_jd_open:
                sz_a = max(sz_a - SZ_STEP, 0.0)
                dmz_jd_a = (min(dmz_jd_a + dmz_jd_step, dmz_jd_open) if dmz_jd_open > dmz_jd_a
                            else max(dmz_jd_a - dmz_jd_step, dmz_jd_open))
                d.ctrl[sz_zf_act] = -sz_a
                d.ctrl[sz_zj_act] = -sz_a
                d.ctrl[dmz_jd_act] = dmz_jd_a
                if not advance(f"RELEASE (cycle {cycle})"):
                    return
            for _ in range(HOLD_STEPS):
                if not advance(f"OPEN (cycle {cycle})"):
                    return
    finally:
        if disp is not None:
            disp.close()
        if ros2_pub is not None:
            ros2_pub.stop()


if __name__ == "__main__":
    main()
