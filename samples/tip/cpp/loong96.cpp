// Loong 96 点触觉阵列落块示例。

#include "tactile_ros2.h"

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

    constexpr int kRows = 12, kCols = 8;
    constexpr int kNumTaxels = kRows * kCols;
    constexpr int kDrawEvery = 15;

    // 沿用目标系统的 tactile_42 通道命名。
    const char *kRos2Topic = "/observation/right_hand/tactile_42";

} // namespace

int main()
{
    // 插件必须先于模型加载。
    const char *so_env  = std::getenv("TIP_LOONG_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_loong.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path  = std::string(TIP_REPO_ROOT) + "/samples/tip/scene/loong96.xml";
    char        error[1000] = "";
    g_model                 = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!g_model) mju_error("模型加载失败: %s", error);
    g_data = mj_makeData(g_model);

    int sensor_id = mj_name2id(g_model, mjOBJ_SENSOR, "loong96_tactile");
    if (sensor_id < 0) mju_error("模型缺少 loong96_tactile 传感器");
    int sensor_adr = g_model->sensor_adr[sensor_id];
    if (g_model->sensor_dim[sensor_id] != kNumTaxels) {
        mju_error("loong96_tactile 维度对不上(期望 %d, 实际 %d)", kNumTaxels,
                  g_model->sensor_dim[sensor_id]);
    }

    tip::TactileRos2Publisher ros2_pub(g_model, g_data,
                                          {
                                              { kRos2Topic, "loong96_tactile", kRows, kCols },
                                          });

    if (!glfwInit()) mju_error("GLFW 初始化失败");
    g_window = glfwCreateWindow(1000, 800, "loong96", nullptr, nullptr);
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

    std::printf("小方块自由落体砸向 loong96 触觉阵列(关掉 3D 窗口结束)……\n\n");
    std::fflush(stdout);

    int k = 0;
    while (!glfwWindowShouldClose(g_window)) {
        mj_step(g_model, g_data);
        ++k;
        if (k % kDrawEvery == 0) {
            Render();
            const mjtNum *sd     = g_data->sensordata + sensor_adr;
            double        total  = mju_sum(sd, kNumTaxels);
            double        peak   = 0.0;
            int           active = 0;
            for (int i = 0; i < kNumTaxels; ++i) {
                if (sd[i] > peak) peak = sd[i];
                if (sd[i] > 1e-6) ++active;
            }
            std::printf("t=%6.2fs  total=%6.2fN  peak=%5.3fN  active=%3d\n", g_data->time, total, peak,
                        active);
            std::fflush(stdout);
        }
    }

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}
