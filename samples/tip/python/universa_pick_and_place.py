#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UR5e 与 Universa 夹爪通过触觉反馈将钢球、钢块从盒 A 搬到盒 B。"""
import os
import sys
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "samples", "common"))
from tactile_ros2 import TactileRos2Publisher  # noqa: E402

SCENE = os.path.join(HERE, "..", "scene", "universa_pick_and_place.xml")
PLUGIN = os.path.join(REPO_ROOT, "lib", "libtip_tactile_universa.so")
ROWS, COLS = 10, 8

# 右、左指分别对应 right_hand / left_hand 的 tactile_54。
ROS2_TOPICS = [
    ("/observation/right_hand/tactile_54", "right_finger_tactile", ROWS, COLS),
    ("/observation/left_hand/tactile_54", "left_finger_tactile", ROWS, COLS),
]

ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
ARM_ACT = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")

# 抓取顺序：（物体 body，目标合力 N）。
OBJECTS = [("ball0", 10.0), ("block0", 10.0), ("ball1", 10.0),
           ("block1", 10.0)]

TABLE_Z = 0.40                 # 台面世界 z
SAFE_Z = 0.54                  # 过渡时 pinch 高度
DROP_Z = 0.45                  # 盒B上方松开高度
BOXB_XY = (0.45, 0.24)         # 盒B中心
R_DOWN = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)  # 夹爪竖直朝下

# pinch 相对物体中心的抓取高度偏移（m）。
GRASP_LIFT = 0.043

JOINT_SPEED = 0.0016           # 关节设定点步长（rad/step）
MOVE_MIN_STEPS = 240           # 单段最少步数
MOVE_MAX_STEPS = 3000          # 单段最多步


CARRY_SPEED_SCALE = 3.0
SETTLE_MOVE = 500              # 每段末尾的最大稳定步数
ARM_VSTOP = 0.02               # 静止速度阈值（rad/s）
ARM_POS_TOL = 0.004            # 到位容差（rad）
GRIP_TARGET_EPS = 0.3
GRIP_STEP = 0.1               # 闭合每步增量 (0-255)
GRIP_MAX = 255.0
MIN_GRIP = 10.0              # 启用力判定的最小闭合量
SETTLE_HOLD = 300


def enable_gravcomp(m):
    """为机械臂及夹爪启用重力补偿，保留物体重力。"""
    base_id = m.body("base").id
    for i in range(m.nbody):
        j = i
        while j > 0:
            if j == base_id:
                m.body_gravcomp[i] = 1.0
                break
            j = m.body_parentid[j]


def ids(m):
    d = {}
    d["arm_q"] = [m.jnt_qposadr[m.joint(n).id] for n in ARM_JOINTS]
    d["arm_dof"] = [m.jnt_dofadr[m.joint(n).id] for n in ARM_JOINTS]
    d["arm_act"] = [m.actuator(n).id for n in ARM_ACT]
    d["grip_act"] = m.actuator("fingers_actuator").id
    d["pinch"] = m.site("pinch").id
    d["rsil"] = m.geom("right_finger_silicone").id
    d["lsil"] = m.geom("left_finger_silicone").id
    d["tactile_l"] = m.sensor("left_finger_tactile").adr[0]
    d["tactile_r"] = m.sensor("right_finger_tactile").adr[0]
    return d


def solve_ik(m, ikd, I, target_pos, R_des, q_seed_full, iters=120):
    """用阻尼最小二乘法求解 pinch 目标位姿对应的六个臂关节角。"""
    ikd.qpos[:] = q_seed_full
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    quat = np.zeros(4)
    aa = np.zeros(3)
    dof = I["arm_dof"]
    qadr = I["arm_q"]
    for _ in range(iters):
        mujoco.mj_forward(m, ikd)
        p = ikd.site_xpos[I["pinch"]]
        R = ikd.site_xmat[I["pinch"]].reshape(3, 3)
        perr = target_pos - p
        Rerr = R_des @ R.T
        mujoco.mju_mat2Quat(quat, Rerr.flatten())
        mujoco.mju_quat2Vel(aa, quat, 1.0)
        err = np.concatenate([perr, aa])
        if np.linalg.norm(perr) < 8e-4 and np.linalg.norm(aa) < 8e-3:
            break
        mujoco.mj_jacSite(m, ikd, jacp, jacr, I["pinch"])
        J = np.vstack([jacp[:, dof], jacr[:, dof]])       # 6x6
        lam = 0.15
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), err)
        dq = np.clip(dq, -0.3, 0.3)
        for k, a in enumerate(qadr):
            ikd.qpos[a] += dq[k]
    return np.array([ikd.qpos[a] for a in qadr]), float(np.linalg.norm(perr))


class Display:
    def __init__(self, m, d):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)

    def update(self):
        self.viewer.sync()

    def alive(self):
        return self.viewer.is_running()

    def close(self):
        self.viewer.close()


class Robot:
    def __init__(self, m, d, I, disp):
        self.m, self.d, self.I, self.disp = m, d, I, disp
        self.ikd = mujoco.MjData(m)
        self.grip = 0.0

    def tactile(self):
        d, I = self.d, self.I
        gl = d.sensordata[I["tactile_l"]:I["tactile_l"] + ROWS * COLS].reshape(ROWS, COLS)
        gr = d.sensordata[I["tactile_r"]:I["tactile_r"] + ROWS * COLS].reshape(ROWS, COLS)
        return gl, gr

    def step(self, phase, draw_every=15, k=[0]):
        m, d, I = self.m, self.d, self.I
        d.ctrl[I["grip_act"]] = self.grip
        mujoco.mj_step(m, d)
        k[0] += 1
        if k[0] % draw_every == 0:
            self.disp.update()
        return self.disp.alive()

    def set_arm_target(self, q):
        for a, qa in zip(self.I["arm_act"], q):
            self.d.ctrl[a] = qa

    def move_to(self, pos, R_des, phase, speed_scale=1.0):
        """用 IK 和 smoothstep 移动到目标位姿；speed_scale > 1 时减速。"""
        q0 = np.array([self.d.qpos[a] for a in self.I["arm_q"]])
        qg, _ = solve_ik(self.m, self.ikd, self.I, np.asarray(pos, float),
                         R_des, self.d.qpos.copy())
        dq = qg - q0
        n = int(np.clip(np.max(np.abs(dq)) / JOINT_SPEED, MOVE_MIN_STEPS, MOVE_MAX_STEPS) * speed_scale)
        for i in range(1, n + 1):
            t = i / n
            s = t * t * (3.0 - 2.0 * t)          # smoothstep，起止速度为零
            self.set_arm_target(q0 + s * dq)
            if not self.step(phase):
                return False
        # 到位且速度足够小时结束稳定阶段。
        self.set_arm_target(qg)
        for _ in range(SETTLE_MOVE):
            if not self.step(phase):
                return False
            cur = np.array([self.d.qpos[a] for a in self.I["arm_q"]])
            if (np.max(np.abs(cur - qg)) < ARM_POS_TOL
                    and np.max(np.abs(self.d.qvel[self.I["arm_dof"]])) < ARM_VSTOP):
                break
        return True

    def set_grip(self, val, phase, hold=200):
        self.grip = val
        for _ in range(hold):
            if not self.step(phase):
                return False
        return True

    def force_close(self, target, phase):
        """闭合至少 MIN_GRIP，双侧合力各达 target/2 或行程到上限后保持。"""
        self.grip = 0.0
        f = 0.0
        for i in range(6000):
            gl, gr = self.tactile()
            f = float(gl.sum() + gr.sum())
            if self.grip >= MIN_GRIP and ((gl.sum() >= target/2 and gr.sum() >= target/2) or self.grip >= GRIP_MAX):
                break
            self.grip = min(self.grip + GRIP_STEP, GRIP_MAX)
            if not self.step(phase):
                return False, f
        for _ in range(SETTLE_HOLD):
            mujoco.mj_step(self.m, self.d)
        return True, float(sum(x.sum() for x in self.tactile()))


def pick_place(m, d, I, disp):
    rob = Robot(m, d, I, disp)
    # 1) 让物品在盒A中落稳
    rob.grip = 0.0
    for _ in range(1500):
        if not rob.step("settling objects in box A"):
            return
    placed = 0
    slots = [(-0.06, -0.06), (0.0, -0.06), (0.06, -0.06),
             (-0.06, 0.06), (0.0, 0.06), (0.06, 0.06)]
    for idx, (name, fN) in enumerate(OBJECTS):
        op = d.body(name).xpos.copy()
        tag = "%s (%d/%d)" % (name, idx + 1, len(OBJECTS))
        # 2) 移到物体上方
        if not rob.move_to([op[0] + 0.005, op[1] - 0.005, SAFE_Z], R_DOWN, "APPROACH " + tag):
            break
        # 3) 张开并下降到抓取高度
        rob.grip = 0.0
        op = d.body(name).xpos.copy()
        gz = max(op[2] + GRASP_LIFT, TABLE_Z + 0.018)
        if not rob.move_to([op[0] + 0.005, op[1] - 0.006, gz], R_DOWN, "DESCEND " + tag):
            break
        # 4) 力闭合抓取
        ok, f = rob.force_close(fN, "GRASP " + tag)
        if not ok:
            break
        # 5) 抬起
        if not rob.move_to([op[0] + 0.005, op[1] - 0.005, SAFE_Z], R_DOWN, "LIFT " + tag):
            break
        # 6) 移到盒B上方对应格
        sx, sy = slots[idx]
        if not rob.move_to([BOXB_XY[0] + sx, BOXB_XY[1] + sy, SAFE_Z], R_DOWN, "CARRY " + tag,
                           speed_scale=CARRY_SPEED_SCALE):
            break
        # 7) 下降并松开
        if not rob.move_to([BOXB_XY[0] + sx, BOXB_XY[1] + sy, DROP_Z], R_DOWN, "PLACE " + tag):
            break
        if not rob.set_grip(0.0, "RELEASE " + tag, hold=400):
            break
        # 8) 抬起回到过渡高度
        if not rob.move_to([BOXB_XY[0] + sx, BOXB_XY[1] + sy, SAFE_Z], R_DOWN, "RETRACT " + tag):
            break
        # 判定是否落进盒B
        fp = d.body(name).xpos
        if abs(fp[0] - BOXB_XY[0]) < 0.12 and abs(fp[1] - BOXB_XY[1]) < 0.12 and fp[2] < 0.55:
            placed += 1
        print("[%d/%d] %-7s grasp_force=%.1fN -> now at (%.2f,%.2f,%.2f)"
              % (idx + 1, len(OBJECTS), name, f, fp[0], fp[1], fp[2]))
    # 结束: 保持画面
    for _ in range(2000):
        if not rob.step("DONE"):
            break
    print("\nplaced %d/%d objects into box B" % (placed, len(OBJECTS)))
    return placed


def main():
    mujoco.mj_loadPluginLibrary(PLUGIN)
    m = mujoco.MjModel.from_xml_path(SCENE)
    enable_gravcomp(m)
    d = mujoco.MjData(m)
    I = ids(m)
    ros2_pub = TactileRos2Publisher(m, d, ROS2_TOPICS)
    disp = Display(m, d)
    pick_place(m, d, I, disp)
    disp.close()
    ros2_pub.stop()


if __name__ == "__main__":
    main()
