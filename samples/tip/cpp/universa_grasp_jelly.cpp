// Universa 夹爪循环抓取、提起并放回柔性果冻。

#include "tactile_ros2.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <string>

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

    constexpr int    kSettleSteps    = 1000;   // 张开状态下的稳定步数
    constexpr double kGripMax        = 210.0;  // 闭合控制上限（0–255）
    constexpr double kGripStep       = 0.15;   // 每步闭合/张开的增量
    constexpr double kForceThreshold = 0.1;    // 每侧软垫的合力阈值（N）
    constexpr double kLiftHeight     = 0.08;   // 提升高度（m）
    constexpr double kLiftStep       = 0.0002; // 每步升降量（m）
    constexpr int    kHoldSteps      = 300;    // 各阶段保持步数
    constexpr int    kDrawEvery      = 15;     // 显示刷新间隔（物理步）

    struct Indices
    {
        int grip_act = -1, lift_act = -1;
        int tactile_l_adr = -1, tactile_r_adr = -1;
    } g_idx;

    void ResolveIndices(const mjModel *m)
    {
        g_idx.grip_act = mj_name2id(m, mjOBJ_ACTUATOR, "fingers_actuator");
        g_idx.lift_act = mj_name2id(m, mjOBJ_ACTUATOR, "lift_actuator");
        int sl         = mj_name2id(m, mjOBJ_SENSOR, "left_finger_tactile");
        int sr         = mj_name2id(m, mjOBJ_SENSOR, "right_finger_tactile");
        if (g_idx.grip_act < 0 || g_idx.lift_act < 0 || sl < 0 || sr < 0) {
            mju_error("模型缺少 fingers_actuator/lift_actuator/left_finger_tactile/right_finger_tactile");
        }
        g_idx.tactile_l_adr = m->sensor_adr[sl];
        g_idx.tactile_r_adr = m->sensor_adr[sr];
    }

    double TactileSum(const mjData *d, int adr) { return mju_sum(d->sensordata + adr, kNumTaxels); }

    // 每 kDrawEvery 步刷新窗口和触觉读数；窗口关闭时返回 false。
    bool Step(const char *phase)
    {
        mj_step(g_model, g_data);

        static int k = 0;
        if (++k % kDrawEvery == 0) {
            Render();
            double gl = TactileSum(g_data, g_idx.tactile_l_adr);
            double gr = TactileSum(g_data, g_idx.tactile_r_adr);
            std::printf("\r%-24s L=%6.2fN  R=%6.2fN   \x1b[K", phase, gl, gr);
            std::fflush(stdout);
        }
        return !glfwWindowShouldClose(g_window);
    }

    void Graspjelly()
    {
        std::printf("果冻在支架上稳定……\n");
        g_data->ctrl[g_idx.grip_act] = 0.0;
        g_data->ctrl[g_idx.lift_act] = 0.0;
        for (int i = 0; i < kSettleSteps; ++i) {
            if (!Step("settling jelly on stand")) return;
        }

        std::printf("循环挤压/提起/放下/松开果冻……(关掉 3D 窗口结束)\n");
        int    cycle = 0;
        double grip  = 0.0;
        double lift  = 0.0;
        while (!glfwWindowShouldClose(g_window)) {
            ++cycle;
            char phase[64];

            // 双侧合力均达阈值或闭合到行程上限后停止。
            double gl = 0.0, gr = 0.0;
            while (grip < kGripMax) {
                grip                         = std::min(grip + kGripStep, kGripMax);
                g_data->ctrl[g_idx.grip_act] = grip;
                std::snprintf(phase, sizeof(phase), "SQUEEZE (cycle %d)", cycle);
                if (!Step(phase)) return;
                gl = TactileSum(g_data, g_idx.tactile_l_adr);
                gr = TactileSum(g_data, g_idx.tactile_r_adr);
                if (gl >= kForceThreshold && gr >= kForceThreshold) break;
            }
            gl                 = TactileSum(g_data, g_idx.tactile_l_adr);
            gr                 = TactileSum(g_data, g_idx.tactile_r_adr);
            const char *reason = grip < kGripMax ? "达到力阈值" : "到达 GRIP_MAX 兜底";
            std::printf("\n[cycle %d] 挤压到 grip=%.0f (%s)  left_total=%.2fN  right_total=%.2fN\n", cycle,
                        grip, reason, gl, gr);
            std::snprintf(phase, sizeof(phase), "HOLD (cycle %d)", cycle);
            for (int i = 0; i < kHoldSteps; ++i) {
                if (!Step(phase)) return;
            }

            // 提起。
            while (lift < kLiftHeight) {
                lift                         = std::min(lift + kLiftStep, kLiftHeight);
                g_data->ctrl[g_idx.lift_act] = lift;
                std::snprintf(phase, sizeof(phase), "LIFT (cycle %d)", cycle);
                if (!Step(phase)) return;
            }
            gl = TactileSum(g_data, g_idx.tactile_l_adr);
            gr = TactileSum(g_data, g_idx.tactile_r_adr);
            std::printf("\n[cycle %d] 提起到 %.0fmm  left_total=%.2fN  right_total=%.2fN\n", cycle,
                        lift * 1000.0, gl, gr);
            std::snprintf(phase, sizeof(phase), "HOLD UP (cycle %d)", cycle);
            for (int i = 0; i < kHoldSteps; ++i) {
                if (!Step(phase)) return;
            }

            // 放回支架。
            while (lift > 0.0) {
                lift                         = std::max(lift - kLiftStep, 0.0);
                g_data->ctrl[g_idx.lift_act] = lift;
                std::snprintf(phase, sizeof(phase), "LOWER (cycle %d)", cycle);
                if (!Step(phase)) return;
            }
            std::snprintf(phase, sizeof(phase), "HOLD DOWN (cycle %d)", cycle);
            for (int i = 0; i < kHoldSteps; ++i) {
                if (!Step(phase)) return;
            }

            // 松开。
            while (grip > 0.0) {
                grip                         = std::max(grip - kGripStep, 0.0);
                g_data->ctrl[g_idx.grip_act] = grip;
                std::snprintf(phase, sizeof(phase), "RELEASE (cycle %d)", cycle);
                if (!Step(phase)) return;
            }
            std::snprintf(phase, sizeof(phase), "OPEN (cycle %d)", cycle);
            for (int i = 0; i < kHoldSteps; ++i) {
                if (!Step(phase)) return;
            }
        }
    }

} // namespace

int main()
{
    // 插件必须先于模型加载。
    const char *so_env  = std::getenv("TIP_UNIVERSA_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_universa.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path =
        std::string(TIP_REPO_ROOT) + "/samples/tip/scene/universa_grasp_jelly.xml";
    char error[1000] = "";
    g_model          = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!g_model) mju_error("模型加载失败: %s", error);
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
    g_window = glfwCreateWindow(1200, 900, "universa grasp jelly", nullptr, nullptr);
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

    Graspjelly();

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    glfwDestroyWindow(g_window);
    glfwTerminate();
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}
