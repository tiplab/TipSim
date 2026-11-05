#include "tactile_ros2.h"

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
    mjModel   *g_model = nullptr;
    mjData    *g_data  = nullptr;
    mjvCamera  g_cam;
    mjvOption  g_opt;
    mjvScene   g_scn;
    mjrContext g_con;

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

    // 手指屈曲方向为关节负角。
    const char *kGraspActuators[] = {
        "sz_zj_actuator",  "sz_zf_actuator",  "zz_zj_actuator",  "zz_zf_actuator",  "wmz_zj_actuator",
        "wmz_zf_actuator", "xmz_zj_actuator", "xmz_zf_actuator", "dmz_jd_actuator",
    };
    constexpr int kNumActuators = sizeof(kGraspActuators) / sizeof(kGraspActuators[0]);

    constexpr double kTarget        = -1.5; // 手指屈曲目标角 (rad)
    constexpr double kCyclePeriod   = 6.0;  // 抓握周期（s）
    constexpr double kRefreshPeriod = 0.2;  // 终端刷新周期（s）

    // 触觉表面名称与阵列尺寸，按终端显示顺序排列。
    struct SurfaceSpec
    {
        const char *key;
        const char *label;
        int         rows, cols;
        const char *topic_id; // 话题后缀；nullptr 时由 key 派生
    };
    // 五个 zd 指端合并到各自 zj 话题，dmz_zz 保留独立话题。
    const SurfaceSpec kSurfaces[] = {
        { "zhangxin_touch_face", "Palm", 8, 14, "61" },
        { "sz_zj_touch_face", "Index tip", 12, 8, "42" },
        { "sz_zf_touch_face", "Index pad", 10, 8, "43" },
        { "sz_zd_touch_face", "Index end", 3, 3, nullptr },
        { "zz_zj_touch_face", "Middle tip", 12, 8, "32" },
        { "zz_zf_touch_face", "Middle pad", 10, 8, "33" },
        { "zz_zd_touch_face", "Middle end", 3, 3, nullptr },
        { "wmz_zj_touch_face", "Ring tip", 12, 8, "22" },
        { "wmz_zf_touch_face", "Ring pad", 10, 8, "23" },
        { "wmz_zd_touch_face", "Ring end", 3, 3, nullptr },
        { "xmz_zj_touch_face", "Pinky tip", 12, 8, "12" },
        { "xmz_zf_touch_face", "Pinky pad", 10, 8, "13" },
        { "xmz_zd_touch_face", "Pinky end", 3, 3, nullptr },
        { "dmz_zj_touch_face", "Thumb tip", 12, 8, "52" },
        { "dmz_zf_touch_face", "Thumb pad", 12, 8, "54" },
        { "dmz_zz_touch_face", "Thumb mid", 3, 3, nullptr },
        { "dmz_zd_touch_face", "Thumb end", 3, 3, nullptr },
    };
    constexpr int kNumSurfaces = sizeof(kSurfaces) / sizeof(kSurfaces[0]);

    // zd 居中补零后与 zj 合并为 15×8，保留原有行列顺序。
    std::vector<tip::TactileTopic> BuildRos2Topics()
    {
        constexpr char                    kFaceSuffix[]  = "_touch_face";
        constexpr int                     kFaceSuffixLen = sizeof(kFaceSuffix) - 1;
        std::vector<tip::TactileTopic> out;
        out.reserve(kNumSurfaces);
        for (int i = 0; i < kNumSurfaces; ++i) {
            const SurfaceSpec& s   = kSurfaces[i];
            std::string        key = s.key;
            if (key.ends_with("_zd_touch_face")) continue;
            std::string suffix     = s.topic_id ? s.topic_id : key.substr(0, key.size() - kFaceSuffixLen);

            tip::TactileTopic t;
            t.topic       = "/observation/right_hand/tactile_" + suffix;
            t.sensor_name = key + "_tactile";
            t.rows        = s.rows;
            t.cols        = s.cols;
            if (key.ends_with("_zj_touch_face")) {
                t.prepend_sensor_name = key.substr(0, key.find("_zj_touch_face")) +
                                        "_zd_touch_face_tactile";
                t.prepend_rows = 3;
                t.prepend_cols = 3;
            }
            out.push_back(std::move(t));
        }
        return out;
    }

    // 原地刷新 17 个触觉表面的终端读数。
    int g_lines_printed = 0;

    void PrintSurfaceTable(const mjModel *m, const mjtNum *sensordata, const int *adr, const char *phase)
    {
        if (g_lines_printed > 0) {
            std::printf("\x1b[%dA", g_lines_printed);
        }
        std::printf("===== %s (t=%.2fs) =====\x1b[K\n", phase, g_data->time);
        double grand = 0.0;
        for (int i = 0; i < kNumSurfaces; ++i) {
            const SurfaceSpec& s   = kSurfaces[i];
            int                dim = s.rows * s.cols;
            double             sum = 0, peak = 0;
            int                active = 0;
            for (int t = 0; t < dim; ++t) {
                double v  = sensordata[adr[i] + t];
                sum      += v;
                if (v > peak) peak = v;
                if (v > 1e-6) ++active;
            }
            grand += sum;
            std::printf("  %-12s [%2dx%-2d]  Sigma=%7.3fN  peak=%6.3fN  active=%3d\x1b[K\n", s.label,
                        s.rows, s.cols, sum, peak, active);
        }
        std::printf("  ----- whole-hand Sigma = %.3f N -----\x1b[K\n", grand);
        g_lines_printed = kNumSurfaces + 2;
        std::fflush(stdout); // 重定向输出时及时刷新
    }

} // namespace

int main()
{
    // 插件必须先于模型加载。
    const char *so_env  = std::getenv("TIP_LOONG_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_loong.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path  = std::string(TIP_REPO_ROOT) + "/samples/inspire/scene/rh56e2_hold_cylinder.xml";
    char        error[1000] = "";
    g_model                 = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!g_model) {
        mju_error("模型加载失败: %s", error);
    }
    g_data = mj_makeData(g_model);

    // 关节下限对应拇指张开姿态。
    int dmz_jd_jid = mj_name2id(g_model, mjOBJ_JOINT, "dmz_jd_joint");
    if (dmz_jd_jid >= 0) {
        g_data->qpos[g_model->jnt_qposadr[dmz_jd_jid]] = g_model->jnt_range[2 * dmz_jd_jid];
    }

    mj_forward(g_model, g_data);

    int sensor_adr[kNumSurfaces];
    for (int i = 0; i < kNumSurfaces; ++i) {
        int sid = mj_name2id(g_model, mjOBJ_SENSOR, (std::string(kSurfaces[i].key) + "_tactile").c_str());
        if (sid < 0) {
            mju_error("模型里找不到 %s_tactile 传感器", kSurfaces[i].key);
        }
        if (g_model->sensor_dim[sid] != kSurfaces[i].rows * kSurfaces[i].cols) {
            mju_error("%s_tactile 维度对不上，模型可能需要用 tools/build_loong_model.py 重新生成",
                      kSurfaces[i].key);
        }
        sensor_adr[i] = g_model->sensor_adr[sid];
    }
    int act_ids[kNumActuators];
    for (int i = 0; i < kNumActuators; ++i) {
        act_ids[i] = mj_name2id(g_model, mjOBJ_ACTUATOR, kGraspActuators[i]);
        if (act_ids[i] < 0) {
            std::fprintf(stderr, "[警告] 找不到执行器 %s\n", kGraspActuators[i]);
        }
    }

    tip::TactileRos2Publisher ros2_pub(g_model, g_data, BuildRos2Topics());

    if (!glfwInit()) {
        mju_error("GLFW 初始化失败");
    }
    GLFWwindow *window = glfwCreateWindow(1200, 900, "loong grasp demo", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&g_cam);
    mjv_defaultOption(&g_opt);
    mjv_defaultScene(&g_scn);
    mjr_defaultContext(&g_con);
    mjv_makeScene(g_model, &g_scn, 2000);
    mjr_makeContext(g_model, &g_con, mjFONTSCALE_150);

    glfwSetMouseButtonCallback(window, MouseButton);
    glfwSetCursorPosCallback(window, MouseMove);
    glfwSetScrollCallback(window, Scroll);

    // 三角波控制抓握与松开，定期刷新触觉读数。
    std::printf("循环抓取/松开圆柱体……(关掉 3D 窗口结束)\n\n");
    std::fflush(stdout);
    double next_refresh = 0.0;
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = g_data->time;
        while (g_data->time - simstart < 1.0 / 60.0) {
            double phase = std::fmod(g_data->time, kCyclePeriod) / kCyclePeriod; // 0..1 一个循环
            double frac  = phase < 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0;      // 0->1->0 三角波
            for (int i = 0; i < kNumActuators; ++i) {
                if (act_ids[i] < 0) continue;
                g_data->ctrl[act_ids[i]] = kTarget * frac;
            }
            mj_step(g_model, g_data);
        }
        if (g_data->time >= next_refresh) {
            double      phase = std::fmod(g_data->time, kCyclePeriod) / kCyclePeriod;
            const char *label = phase < 0.5 ? "grasping" : "releasing";
            PrintSurfaceTable(g_model, g_data->sensordata, sensor_adr, label);
            next_refresh = g_data->time + kRefreshPeriod;
        }

        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(g_model, g_data, &g_opt, nullptr, &g_cam, mjCAT_ALL, &g_scn);
        mjr_render(viewport, &g_scn, &g_con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}
