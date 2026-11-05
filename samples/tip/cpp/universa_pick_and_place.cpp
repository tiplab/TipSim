// UR5e 与 Universa 夹爪通过触觉反馈完成取放。

#include "tactile_ros2.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

namespace
{
    // GLFW 回调共享的可视化状态。
    mjModel    *g_model  = nullptr;
    mjData     *g_data   = nullptr;
    GLFWwindow *g_window = nullptr;
    mjvCamera   g_cam;
    mjvOption   g_opt;
    mjvScene    g_scn;
    mjrContext  g_con;

    bool   g_button_left = false, g_button_middle = false, g_button_right = false;
    double g_lastx = 0, g_lasty = 0;

    void MouseButton(GLFWwindow *window, int /*button*/, int /*act*/, int /*mods*/)
    {
        g_button_left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        g_button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        g_button_right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        glfwGetCursorPos(window, &g_lastx, &g_lasty);
    }

    void MouseMove(GLFWwindow *window, double xpos, double ypos)
    {
        if (!g_button_left && !g_button_middle && !g_button_right) return;
        double dx = xpos - g_lastx, dy = ypos - g_lasty;
        g_lastx = xpos;
        g_lasty = ypos;
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        bool     mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                             glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        mjtMouse action;
        if (g_button_right) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (g_button_left) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else {
            action = mjMOUSE_ZOOM;
        }
#if mjVERSION_HEADER >= 3011000
        mjv_moveCamera(g_model, action, dx / height, dy / height, &g_cam);
#else
        mjv_moveCamera(g_model, action, dx / height, dy / height, &g_scn, &g_cam);
#endif
    }

    void Scroll(GLFWwindow * /*window*/, double /*xoffset*/, double yoffset)
    {
#if mjVERSION_HEADER >= 3011000
        mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &g_cam);
#else
        mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &g_scn, &g_cam);
#endif
    }

    void Render()
    {
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(g_window, &viewport.width, &viewport.height);
        mjv_updateScene(g_model, g_data, &g_opt, nullptr, &g_cam, mjCAT_ALL, &g_scn);
        mjr_render(viewport, &g_scn, &g_con);
        glfwSwapBuffers(g_window);
        glfwPollEvents();
    }

    constexpr int kRows = 10, kCols = 8;
    constexpr int kNumTaxels = kRows * kCols;

    constexpr int     kNumArmJoints                = 6;
    const char *const kArmJoints[kNumArmJoints]    = { "shoulder_pan_joint", "shoulder_lift_joint",
                                                       "elbow_joint",        "wrist_1_joint",
                                                       "wrist_2_joint",      "wrist_3_joint" };
    const char *const kArmActuators[kNumArmJoints] = { "shoulder_pan", "shoulder_lift", "elbow",
                                                       "wrist_1",      "wrist_2",       "wrist_3" };

    struct ObjectSpec
    {
        const char *name;
        double      grip_force_n;
    };
    const ObjectSpec kObjects[] = {
        { "ball0", 10.0 },
        { "block0", 10.0 },
        { "ball1", 10.0 },
        { "block1", 10.0 },
    };
    constexpr int kNumObjects = sizeof(kObjects) / sizeof(kObjects[0]);

    constexpr double kTableZ = 0.40;               // 台面世界 z
    constexpr double kSafeZ  = 0.54;               // 过渡时 pinch 高度
    constexpr double kDropZ  = 0.45;               // 盒B上方松开高度
    constexpr double kBoxBX = 0.45, kBoxBY = 0.24; // 盒B中心
    // 夹爪竖直朝下(行主序 3x3)
    const double     kRDown[9] = { 1, 0, 0, 0, -1, 0, 0, 0, -1 };

    // pinch 相对物体中心的抓取高度偏移（m）。
    constexpr double kGraspLift = 0.043;

    constexpr double kJointSpeed      = 0.0016; // 关节设定点步长（rad/step）
    constexpr int    kMoveMinSteps    = 240;
    constexpr int    kMoveMaxSteps    = 3000;
    constexpr double kCarrySpeedScale = 3.0;   // 搬运阶段减速倍数
    constexpr int    kSettleMove      = 500;
    constexpr double kArmVstop        = 0.02;  // 静止速度阈值（rad/s）
    constexpr double kArmPosTol       = 0.004; // 到位容差（rad）
    constexpr double kGripStep        = 0.1;   // 闭合每步增量 (0-255)
    constexpr double kGripMax         = 255.0;
    constexpr double kMinGrip         = 10.0;  // 启用力判定的最小闭合量
    constexpr int    kSettleHold      = 300;

    const double kSlots[6][2] = {
        { -0.06, -0.06 }, { 0.0, -0.06 }, { 0.06, -0.06 }, { -0.06, 0.06 }, { 0.0, 0.06 }, { 0.06, 0.06 },
    };

    struct Indices
    {
        int arm_qpos[kNumArmJoints];
        int arm_dof[kNumArmJoints];
        int arm_act[kNumArmJoints];
        int grip_act      = -1;
        int pinch_site    = -1;
        int tactile_l_adr = -1, tactile_r_adr = -1;
    } g_idx;

    void EnableGravcomp(mjModel *m)
    {
        int base_id = mj_name2id(m, mjOBJ_BODY, "base");
        for (int i = 0; i < m->nbody; ++i) {
            for (int j = i; j > 0; j = m->body_parentid[j]) {
                if (j == base_id) {
                    m->body_gravcomp[i] = 1.0;
                    break;
                }
            }
        }
    }

    void ResolveIndices(const mjModel *m)
    {
        for (int i = 0; i < kNumArmJoints; ++i) {
            int jid = mj_name2id(m, mjOBJ_JOINT, kArmJoints[i]);
            if (jid < 0) mju_error("找不到关节 %s", kArmJoints[i]);
            g_idx.arm_qpos[i] = m->jnt_qposadr[jid];
            g_idx.arm_dof[i]  = m->jnt_dofadr[jid];
            g_idx.arm_act[i]  = mj_name2id(m, mjOBJ_ACTUATOR, kArmActuators[i]);
        }
        g_idx.grip_act   = mj_name2id(m, mjOBJ_ACTUATOR, "fingers_actuator");
        g_idx.pinch_site = mj_name2id(m, mjOBJ_SITE, "pinch");
        int sl           = mj_name2id(m, mjOBJ_SENSOR, "left_finger_tactile");
        int sr           = mj_name2id(m, mjOBJ_SENSOR, "right_finger_tactile");
        if (g_idx.grip_act < 0 || g_idx.pinch_site < 0 || sl < 0 || sr < 0) {
            mju_error("模型缺少 fingers_actuator/pinch/left_finger_tactile/right_finger_tactile");
        }
        g_idx.tactile_l_adr = m->sensor_adr[sl];
        g_idx.tactile_r_adr = m->sensor_adr[sr];
    }

    double TactileSum(const mjData *d, int adr) { return mju_sum(d->sensordata + adr, kNumTaxels); }

    // 阻尼最小二乘 IK：仅调整六个臂关节，姿态矩阵采用行主序。
    void SolveIK(const mjModel *m, mjData *ikd, const double target_pos[3], const double R_des[9],
                 const mjtNum *q_seed_full, double out_q[kNumArmJoints], int iters = 120)
    {
        mju_copy(ikd->qpos, q_seed_full, m->nq);
        std::vector<mjtNum> jacp(3 * m->nv), jacr(3 * m->nv);

        for (int iter = 0; iter < iters; ++iter) {
            mj_forward(m, ikd);
            const mjtNum *p = ikd->site_xpos + 3 * g_idx.pinch_site;
            const mjtNum *R = ikd->site_xmat + 9 * g_idx.pinch_site;

            double perr[3];
            mju_sub3(perr, target_pos, p);

            double Rt[9], Rerr[9], quat[4], aa[3];
            mju_transpose(Rt, R, 3, 3);
            mju_mulMatMat(Rerr, R_des, Rt, 3, 3, 3);
            mju_mat2Quat(quat, Rerr);
            mju_quat2Vel(aa, quat, 1.0);

            if (mju_norm3(perr) < 8e-4 && mju_norm3(aa) < 8e-3) break;

            mj_jacSite(m, ikd, jacp.data(), jacr.data(), g_idx.pinch_site);

            // 雅可比前三行为位置、后三行为姿态，列按 arm_dof 排列。
            double J[36]; // 行主序
            for (int c = 0; c < kNumArmJoints; ++c) {
                int dof = g_idx.arm_dof[c];
                for (int r = 0; r < 3; ++r) {
                    J[r * kNumArmJoints + c]       = jacp[r * m->nv + dof];
                    J[(r + 3) * kNumArmJoints + c] = jacr[r * m->nv + dof];
                }
            }
            const double err[6] = { perr[0], perr[1], perr[2], aa[0], aa[1], aa[2] };

            // dq = J' * (J*J' + lam^2*I)^-1 * err
            constexpr double lam = 0.15;
            double           M[36];
            mju_mulMatMatT(M, J, J, 6, 6, 6);
            for (int i = 0; i < 6; ++i)
                M[i * 6 + i] += lam * lam;
            mju_cholFactor(M, 6, 1e-12);
            double y[6];
            mju_cholSolve(y, M, err, 6);
            double dq[6];
            mju_mulMatTVec(dq, J, y, 6, 6);

            for (int i = 0; i < 6; ++i) {
                dq[i]                         = std::clamp(dq[i], -0.3, 0.3);
                ikd->qpos[g_idx.arm_qpos[i]] += dq[i];
            }
        }
        for (int i = 0; i < kNumArmJoints; ++i)
            out_q[i] = ikd->qpos[g_idx.arm_qpos[i]];
    }

    struct Robot
    {
        mjData *ikd  = nullptr;
        double  grip = 0.0;
    } g_robot;

    // 每 15 步刷新窗口和触觉读数；窗口关闭时返回 false。
    bool Step(const char *phase)
    {
        g_data->ctrl[g_idx.grip_act] = g_robot.grip;
        mj_step(g_model, g_data);

        static int k = 0;
        if (++k % 15 == 0) {
            Render();
            double gl = TactileSum(g_data, g_idx.tactile_l_adr);
            double gr = TactileSum(g_data, g_idx.tactile_r_adr);
            std::printf("\r%-20s L=%6.2fN  R=%6.2fN   \x1b[K", phase, gl, gr);
            std::fflush(stdout);
        }
        return !glfwWindowShouldClose(g_window);
    }

    void SetArmTarget(const double q[kNumArmJoints])
    {
        for (int i = 0; i < kNumArmJoints; ++i)
            g_data->ctrl[g_idx.arm_act[i]] = q[i];
    }

    // IK 配合 smoothstep 移动到目标；speed_scale > 1 时减速。
    bool MoveTo(const double pos[3], const double R_des[9], const char *phase, double speed_scale = 1.0)
    {
        double q0[kNumArmJoints], qg[kNumArmJoints], dq[kNumArmJoints];
        for (int i = 0; i < kNumArmJoints; ++i)
            q0[i] = g_data->qpos[g_idx.arm_qpos[i]];
        SolveIK(g_model, g_robot.ikd, pos, R_des, g_data->qpos, qg);

        double max_dq = 0.0;
        for (int i = 0; i < kNumArmJoints; ++i) {
            dq[i]  = qg[i] - q0[i];
            max_dq = std::max(max_dq, std::abs(dq[i]));
        }
        int n = static_cast<int>(
            std::clamp(max_dq / kJointSpeed, double(kMoveMinSteps), double(kMoveMaxSteps)) * speed_scale);

        double q[kNumArmJoints];
        for (int i = 1; i <= n; ++i) {
            double t = static_cast<double>(i) / n;
            double s = t * t * (3.0 - 2.0 * t); // smoothstep，起止速度为零
            for (int j = 0; j < kNumArmJoints; ++j)
                q[j] = q0[j] + s * dq[j];
            SetArmTarget(q);
            if (!Step(phase)) return false;
        }
        // 到位且速度足够小时结束稳定阶段。
        SetArmTarget(qg);
        for (int i = 0; i < kSettleMove; ++i) {
            if (!Step(phase)) return false;
            double max_perr = 0.0, max_vel = 0.0;
            for (int j = 0; j < kNumArmJoints; ++j) {
                max_perr = std::max(max_perr, std::abs(g_data->qpos[g_idx.arm_qpos[j]] - qg[j]));
                max_vel  = std::max(max_vel, std::abs(g_data->qvel[g_idx.arm_dof[j]]));
            }
            if (max_perr < kArmPosTol && max_vel < kArmVstop) break;
        }
        return true;
    }

    bool SetGrip(double val, const char *phase, int hold = 200)
    {
        g_robot.grip = val;
        for (int i = 0; i < hold; ++i) {
            if (!Step(phase)) return false;
        }
        return true;
    }

    // 闭合至少 kMinGrip，双侧合力各达目标一半或行程到上限后保持。
    bool ForceClose(double target, const char *phase, double *out_force)
    {
        g_robot.grip = 0.0;
        double f     = 0.0;
        for (int i = 0; i < 6000; ++i) {
            double gl = TactileSum(g_data, g_idx.tactile_l_adr);
            double gr = TactileSum(g_data, g_idx.tactile_r_adr);
            f         = gl + gr;
            if (g_robot.grip >= kMinGrip &&
                ((gl >= target / 2 && gr >= target / 2) || g_robot.grip >= kGripMax)) {
                break;
            }
            g_robot.grip = std::min(g_robot.grip + kGripStep, kGripMax);
            if (!Step(phase)) {
                *out_force = f;
                return false;
            }
        }
        for (int i = 0; i < kSettleHold; ++i)
            mj_step(g_model, g_data);
        *out_force = TactileSum(g_data, g_idx.tactile_l_adr) + TactileSum(g_data, g_idx.tactile_r_adr);
        return true;
    }

    int PickPlace()
    {
        g_robot.ikd  = mj_makeData(g_model);
        g_robot.grip = 0.0;

        // 1) 让物品在盒A中落稳
        for (int i = 0; i < 1500; ++i) {
            if (!Step("settling objects in box A")) {
                mj_deleteData(g_robot.ikd);
                return 0;
            }
        }

        int placed = 0;
        for (int idx = 0; idx < kNumObjects; ++idx) {
            const ObjectSpec& obj  = kObjects[idx];
            int               body = mj_name2id(g_model, mjOBJ_BODY, obj.name);
            mjtNum            op[3];
            // 保存位置副本，避免随 mj_step 更新。
            mju_copy3(op, g_data->xpos + 3 * body);
            char              tag[64];
            std::snprintf(tag, sizeof(tag), "%s (%d/%d)", obj.name, idx + 1, kNumObjects);

            // 2) 移到物体上方
            double pos[3] = { op[0] + 0.005, op[1] - 0.005, kSafeZ };
            char   phase[96];
            std::snprintf(phase, sizeof(phase), "APPROACH %s", tag);
            if (!MoveTo(pos, kRDown, phase)) break;

            // 3) 张开并下降到抓取高度
            g_robot.grip = 0.0;
            mju_copy3(op, g_data->xpos + 3 * body); // 接近后更新副本, 供下降和抬起共用
            double gz      = std::max(op[2] + kGraspLift, kTableZ + 0.018);
            double dpos[3] = { op[0] + 0.005, op[1] - 0.006, gz };
            std::snprintf(phase, sizeof(phase), "DESCEND %s", tag);
            if (!MoveTo(dpos, kRDown, phase)) break;

            // 4) 力闭合抓取
            double f = 0.0;
            std::snprintf(phase, sizeof(phase), "GRASP %s", tag);
            if (!ForceClose(obj.grip_force_n, phase, &f)) break;

            // 5) 抬起
            double lift[3] = { op[0] + 0.005, op[1] - 0.005, kSafeZ };
            std::snprintf(phase, sizeof(phase), "LIFT %s", tag);
            if (!MoveTo(lift, kRDown, phase)) break;

            // 6) 移到盒B上方对应格
            double sx = kSlots[idx][0], sy = kSlots[idx][1];
            double carry[3] = { kBoxBX + sx, kBoxBY + sy, kSafeZ };
            std::snprintf(phase, sizeof(phase), "CARRY %s", tag);
            if (!MoveTo(carry, kRDown, phase, kCarrySpeedScale)) break;

            // 7) 下降并松开
            double place[3] = { kBoxBX + sx, kBoxBY + sy, kDropZ };
            std::snprintf(phase, sizeof(phase), "PLACE %s", tag);
            if (!MoveTo(place, kRDown, phase)) break;
            std::snprintf(phase, sizeof(phase), "RELEASE %s", tag);
            if (!SetGrip(0.0, phase, 400)) break;

            // 8) 抬起回到过渡高度
            double retract[3] = { kBoxBX + sx, kBoxBY + sy, kSafeZ };
            std::snprintf(phase, sizeof(phase), "RETRACT %s", tag);
            if (!MoveTo(retract, kRDown, phase)) break;

            // 判定是否落进盒B
            const mjtNum *fp = g_data->xpos + 3 * body;
            if (std::abs(fp[0] - kBoxBX) < 0.12 && std::abs(fp[1] - kBoxBY) < 0.12 && fp[2] < 0.55) {
                ++placed;
            }
            std::printf("\n[%d/%d] %-7s grasp_force=%.1fN -> now at (%.2f,%.2f,%.2f)\n", idx + 1,
                        kNumObjects, obj.name, f, fp[0], fp[1], fp[2]);
        }

        // 结束: 保持画面
        for (int i = 0; i < 2000; ++i) {
            if (!Step("DONE")) break;
        }
        std::printf("\n\nplaced %d/%d objects into box B\n", placed, kNumObjects);
        mj_deleteData(g_robot.ikd);
        return placed;
    }

} // namespace

int main()
{
    // 插件必须先于模型加载。
    const char *so_env  = std::getenv("TIP_UNIVERSA_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_universa.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path =
        std::string(TIP_REPO_ROOT) + "/samples/tip/scene/universa_pick_and_place.xml";
    char error[1000] = "";
    g_model          = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!g_model) mju_error("模型加载失败: %s", error);
    EnableGravcomp(g_model);
    g_data = mj_makeData(g_model);
    ResolveIndices(g_model);

    // 右、左指分别对应 right_hand / left_hand 的 tactile_54。
    tip::TactileRos2Publisher ros2_pub(
        g_model, g_data,
        {
            { "/observation/right_hand/tactile_54", "right_finger_tactile", kRows, kCols },
            { "/observation/left_hand/tactile_54", "left_finger_tactile", kRows, kCols },
        });

    if (!glfwInit()) mju_error("GLFW 初始化失败");
    g_window = glfwCreateWindow(1200, 900, "Universa pick and place", nullptr, nullptr);
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    mjv_defaultFreeCamera(g_model, &g_cam);
    mjv_defaultOption(&g_opt);
    mjv_defaultScene(&g_scn);
    mjr_defaultContext(&g_con);
    mjv_makeScene(g_model, &g_scn, 2000);
    mjr_makeContext(g_model, &g_con, mjFONTSCALE_150);

    glfwSetMouseButtonCallback(g_window, MouseButton);
    glfwSetCursorPosCallback(g_window, MouseMove);
    glfwSetScrollCallback(g_window, Scroll);

    std::printf("UR5e 工业取放(关掉 3D 窗口结束)……\n");
    PickPlace();

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    glfwDestroyWindow(g_window);
    glfwTerminate();
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}
