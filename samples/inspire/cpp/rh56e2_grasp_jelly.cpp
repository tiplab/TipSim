#include "tactile_ros2.h"

#include <algorithm>
#include <cmath>
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

    // 食指、拇指各有指腹、指尖、指端三块触觉表面。
    struct SurfaceSpec
    {
        const char *key;
        int         rows, cols;
    };
    const SurfaceSpec kIndexSurfaces[] = {
        { "sz_zf_touch_face", 10, 8 },
        { "sz_zj_touch_face", 12, 8 },
        { "sz_zd_touch_face", 3, 3 },
    };
    const SurfaceSpec kThumbSurfaces[] = {
        { "dmz_zf_touch_face", 12, 8 },
        { "dmz_zj_touch_face", 12, 8 },
        { "dmz_zd_touch_face", 3, 3 },
    };
    constexpr int kNumIndexSurf = sizeof(kIndexSurfaces) / sizeof(kIndexSurfaces[0]);
    constexpr int kNumThumbSurf = sizeof(kThumbSurfaces) / sizeof(kThumbSurfaces[0]);

    constexpr int    kSettleSteps    = 1000;     // 张开状态下的稳定步数
    constexpr double kDmzJd          = 0.0;      // 拇指根部闭合目标角（rad）
    constexpr double kSzMax          = 1.35;     // 食指屈曲上限（rad）
    constexpr double kSzStep         = 0.0003;   // 食指每步屈曲量（rad）
    constexpr double kForceThreshold = 1.0;      // 单指三块表面的合力阈值（N）
    constexpr double kLiftHeight     = 0.05;     // 提升高度（m）
    constexpr double kLiftStep       = 0.000015; // 每步升降量（m）
    constexpr int    kHoldSteps      = 300;      // 各阶段保持步数
    constexpr int    kDrawEvery      = 15;       // 显示刷新间隔（物理步）

    struct Indices
    {
        int sz_zf_act = -1, sz_zj_act = -1, dmz_jd_act = -1, lift_act = -1;
        int index_adr[kNumIndexSurf], index_dim[kNumIndexSurf];
        int thumb_adr[kNumThumbSurf], thumb_dim[kNumThumbSurf];
    } g_idx;

    void ResolveIndices(const mjModel *m)
    {
        g_idx.sz_zf_act  = mj_name2id(m, mjOBJ_ACTUATOR, "sz_zf_actuator");
        g_idx.sz_zj_act  = mj_name2id(m, mjOBJ_ACTUATOR, "sz_zj_actuator");
        g_idx.dmz_jd_act = mj_name2id(m, mjOBJ_ACTUATOR, "dmz_jd_actuator");
        g_idx.lift_act   = mj_name2id(m, mjOBJ_ACTUATOR, "lift_actuator");
        if (g_idx.sz_zf_act < 0 || g_idx.sz_zj_act < 0 || g_idx.dmz_jd_act < 0 || g_idx.lift_act < 0) {
            mju_error("模型缺少 sz_zf_actuator/sz_zj_actuator/dmz_jd_actuator/lift_actuator");
        }
        for (int i = 0; i < kNumIndexSurf; ++i) {
            const SurfaceSpec& s   = kIndexSurfaces[i];
            int                sid = mj_name2id(m, mjOBJ_SENSOR, (std::string(s.key) + "_tactile").c_str());
            if (sid < 0) mju_error("模型缺少 %s_tactile 传感器", s.key);
            g_idx.index_adr[i] = m->sensor_adr[sid];
            g_idx.index_dim[i] = s.rows * s.cols;
        }
        for (int i = 0; i < kNumThumbSurf; ++i) {
            const SurfaceSpec& s   = kThumbSurfaces[i];
            int                sid = mj_name2id(m, mjOBJ_SENSOR, (std::string(s.key) + "_tactile").c_str());
            if (sid < 0) mju_error("模型缺少 %s_tactile 传感器", s.key);
            g_idx.thumb_adr[i] = m->sensor_adr[sid];
            g_idx.thumb_dim[i] = s.rows * s.cols;
        }
    }

    double IndexTotal(const mjData *d)
    {
        double sum = 0.0;
        for (int i = 0; i < kNumIndexSurf; ++i)
            sum += mju_sum(d->sensordata + g_idx.index_adr[i], g_idx.index_dim[i]);
        return sum;
    }

    double ThumbTotal(const mjData *d)
    {
        double sum = 0.0;
        for (int i = 0; i < kNumThumbSurf; ++i)
            sum += mju_sum(d->sensordata + g_idx.thumb_adr[i], g_idx.thumb_dim[i]);
        return sum;
    }

    // 每 kDrawEvery 步刷新窗口和触觉读数；窗口关闭时返回 false。
    bool Step(const char *phase)
    {
        mj_step(g_model, g_data);

        static int k = 0;
        if (++k % kDrawEvery == 0) {
            Render();
            std::printf("\r%-24s index=%6.2fN  thumb=%6.2fN   \x1b[K", phase, IndexTotal(g_data),
                        ThumbTotal(g_data));
            std::fflush(stdout);
        }
        return !glfwWindowShouldClose(g_window);
    }

    void Graspjelly(double dmz_jd_open)
    {
        std::printf("果冻在支架上稳定……\n");
        g_data->ctrl[g_idx.sz_zf_act] = 0.0;
        g_data->ctrl[g_idx.sz_zj_act] = 0.0;
        g_data->ctrl[g_idx.lift_act]  = 0.0;
        for (int i = 0; i < kSettleSteps; ++i) {
            if (!Step("settling jelly on stand")) return;
        }

        std::printf("循环捏紧/提起/放下/松开果冻……(关掉 3D 窗口结束)\n");
        int    cycle       = 0;
        double sz_a        = 0.0;
        double dmz_jd_a    = dmz_jd_open;
        // 按食指行程估算拇指步长，两指仍独立控制。
        double dmz_jd_step = std::abs(kDmzJd - dmz_jd_open) / (kSzMax / kSzStep);
        double lift        = 0.0;

        while (!glfwWindowShouldClose(g_window)) {
            ++cycle;
            char phase[64];

            // 两指独立闭合，各自达到力阈值或行程上限后停止。
            bool sz_done = false, dmz_done = false;
            while (!(sz_done && dmz_done)) {
                if (!sz_done) {
                    sz_a                          = std::min(sz_a + kSzStep, kSzMax);
                    g_data->ctrl[g_idx.sz_zf_act] = -sz_a;
                    g_data->ctrl[g_idx.sz_zj_act] = -sz_a;
                }
                if (!dmz_done) {
                    dmz_jd_a = dmz_jd_a < kDmzJd ? std::min(dmz_jd_a + dmz_jd_step, kDmzJd)
                                                 : std::max(dmz_jd_a - dmz_jd_step, kDmzJd);
                    g_data->ctrl[g_idx.dmz_jd_act] = dmz_jd_a;
                }
                std::snprintf(phase, sizeof(phase), "SQUEEZE (cycle %d)", cycle);
                if (!Step(phase)) return;
                double index_now = IndexTotal(g_data);
                double thumb_now = ThumbTotal(g_data);
                if (!sz_done && (index_now >= kForceThreshold || sz_a >= kSzMax)) sz_done = true;
                if (!dmz_done && (thumb_now >= kForceThreshold || dmz_jd_a == kDmzJd)) dmz_done = true;
            }
            const char *sz_reason  = sz_a < kSzMax ? "达到力阈值" : "到达 SZ_MAX 兜底";
            const char *dmz_reason = dmz_jd_a != kDmzJd ? "达到力阈值" : "到达 DMZ_JD 兜底";
            std::printf(
                "\n[cycle %d] 食指屈曲到 %.2frad(%s) index=%.2fN  拇指合拢到 %.2frad(%s) thumb=%.2fN\n",
                cycle, sz_a, sz_reason, IndexTotal(g_data), dmz_jd_a, dmz_reason, ThumbTotal(g_data));
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

            // 两指独立松开，回到初始张开姿态。
            while (sz_a > 0.0 || dmz_jd_a != dmz_jd_open) {
                sz_a     = std::max(sz_a - kSzStep, 0.0);
                dmz_jd_a = dmz_jd_a < dmz_jd_open ? std::min(dmz_jd_a + dmz_jd_step, dmz_jd_open)
                                                  : std::max(dmz_jd_a - dmz_jd_step, dmz_jd_open);
                g_data->ctrl[g_idx.sz_zf_act]  = -sz_a;
                g_data->ctrl[g_idx.sz_zj_act]  = -sz_a;
                g_data->ctrl[g_idx.dmz_jd_act] = dmz_jd_a;
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
    const char *so_env  = std::getenv("TIP_LOONG_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_loong.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path =
        std::string(TIP_REPO_ROOT) + "/samples/inspire/scene/rh56e2_grasp_jelly.xml";
    char error[1000] = "";
    g_model          = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!g_model) mju_error("模型加载失败: %s", error);
    g_data = mj_makeData(g_model);
    ResolveIndices(g_model);

    // zd 居中补零后与 zj 合并为 15×8，保留原有行列顺序。
    tip::TactileRos2Publisher ros2_pub(
        g_model, g_data,
        {
            { "/observation/right_hand/tactile_43", "sz_zf_touch_face_tactile", 10, 8 },
            { "/observation/right_hand/tactile_42", "sz_zj_touch_face_tactile", 12, 8,
              "sz_zd_touch_face_tactile", 3, 3 },
            { "/observation/right_hand/tactile_54", "dmz_zf_touch_face_tactile", 12, 8 },
            { "/observation/right_hand/tactile_52", "dmz_zj_touch_face_tactile", 12, 8,
              "dmz_zd_touch_face_tactile", 3, 3 },
        });

    // 关节下限对应拇指张开姿态。
    int dmz_jd_jid = mj_name2id(g_model, mjOBJ_JOINT, "dmz_jd_joint");
    if (dmz_jd_jid < 0) mju_error("模型缺少 dmz_jd_joint 关节");
    double dmz_jd_open                             = g_model->jnt_range[2 * dmz_jd_jid];
    g_data->qpos[g_model->jnt_qposadr[dmz_jd_jid]] = dmz_jd_open;
    g_data->ctrl[g_idx.dmz_jd_act]                 = dmz_jd_open;
    mj_forward(g_model, g_data);

    if (!glfwInit()) mju_error("GLFW 初始化失败");
    g_window = glfwCreateWindow(1200, 900, "inspire grasp jelly", nullptr, nullptr);
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&g_cam);
    mjv_defaultOption(&g_opt);
    mjv_defaultScene(&g_scn);
    mjr_defaultContext(&g_con);
    mjv_makeScene(g_model, &g_scn, 2000);
    mjr_makeContext(g_model, &g_con, mjFONTSCALE_150);

    glfwSetMouseButtonCallback(g_window, MouseButton);
    glfwSetCursorPosCallback(g_window, MouseMove);
    glfwSetScrollCallback(g_window, Scroll);

    Graspjelly(dmz_jd_open);

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}
