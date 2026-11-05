// Universa 单指按压示例，支持球体、长方体、正方体和螺母。

#include "tactile_ros2.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>
#include <tinyxml2.h>

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
    constexpr int kDrawEvery = 15;

    // 施加力线性增加至上限后保持；传感器量程由 XML 配置。
    constexpr double kFMax     = 35.0;
    constexpr double kRampTime = 8.0;

    double AppliedForce(double t) { return std::min(kFMax, kFMax * t / kRampTime); }

    // 沿用目标系统的 tactile_54 通道命名。
    const char *kRos2Topic = "/observation/right_hand/tactile_54";

    void PrintUsage(const char *program)
    {
        std::printf("用法: %s [--object {sphere,cuboid,cube,nut}]\n"
                    "  sphere 球体(默认), cuboid 长方体, cube 正方体, nut 螺母\n"
                    "  -h, --help  显示帮助\n", program);
    }

    std::string ParseObject(int argc, char **argv)
    {
        std::string object = "sphere";
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "-h" || arg == "--help") {
                PrintUsage(argv[0]);
                return {};
            }
            if (arg == "--object") {
                if (++i == argc) throw std::invalid_argument("--object 缺少参数");
                object = argv[i];
            }
            else if (arg.starts_with("--object=")) object = arg.substr(9);
            else throw std::invalid_argument("未知参数: " + arg);
        }
        if (object != "sphere" && object != "cuboid" && object != "cube" && object != "nut")
            throw std::invalid_argument("未知按压对象: " + object + "; 可选 sphere, cuboid, cube, nut");
        return object;
    }

    mjModel *LoadModel(const std::string& path, const std::string& object)
    {
        // 编译前选择压头，保留安装和接触顺序。
        tinyxml2::XMLDocument xml;
        if (xml.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
            throw std::runtime_error("场景读取失败: " + std::string(xml.ErrorStr()));
        auto *root = xml.RootElement();
        auto *compiler = root->FirstChildElement("compiler");
        auto meshdir = (std::filesystem::path(path).parent_path() / compiler->Attribute("meshdir")).lexically_normal();
        compiler->SetAttribute("meshdir", meshdir.string().c_str());

        auto *probe = root->FirstChildElement("worldbody")->FirstChildElement("body");
        while (probe && !probe->Attribute("name", "probe")) probe = probe->NextSiblingElement("body");
        if (!probe) throw std::runtime_error("场景缺少 probe 压头");
        if (object == "nut") {
            probe->DeleteChild(probe->FirstChildElement("geom"));
            for (auto *mesh = root->FirstChildElement("asset")->FirstChildElement("mesh");
                 mesh; mesh = mesh->NextSiblingElement("mesh")) {
                const char *name = mesh->Attribute("name");
                if (!name || !std::string(name).starts_with("probe_nut_hull")) continue;
                auto *geom = xml.NewElement("geom");
                geom->SetAttribute("name", name);
                geom->SetAttribute("mesh", name);
                geom->SetAttribute("class", "probe_nut");
                probe->InsertEndChild(geom);
            }
        }
        else probe->FirstChildElement("geom")->SetAttribute("class", ("probe_" + object).c_str());

        std::string names;
        for (auto *geom = probe->FirstChildElement("geom"); geom; geom = geom->NextSiblingElement("geom")) {
            if (!names.empty()) names += ' ';
            names += geom->Attribute("name");
        }
        auto *rigid = root->FirstChildElement("custom")->FirstChildElement("text");
        while (rigid && !rigid->Attribute("name", "universa_rigid")) rigid = rigid->NextSiblingElement("text");
        if (!rigid) throw std::runtime_error("场景缺少 universa_rigid 配置");
        rigid->SetAttribute("data", names.c_str());

        // 绝对网格路径支持从内存加载 XML。
        tinyxml2::XMLPrinter printer;
        xml.Print(&printer);
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        int status = mj_addBufferVFS(&vfs, "universa_finger_selected.xml", printer.CStr(), printer.CStrSize());
        char error[1000] = "";
        mjModel *model = status == 0 ? mj_loadXML("universa_finger_selected.xml", &vfs, error, sizeof(error)) : nullptr;
        mj_deleteVFS(&vfs);
        if (!model) throw std::runtime_error(status ? "无法创建场景 XML 缓冲区" : "模型加载失败: " + std::string(error));
        return model;
    }

} // namespace

static int RunSample(const std::string& object)
{
    // 插件必须先于模型加载。
    const char *so_env  = std::getenv("TIP_UNIVERSA_PLUGIN");
    std::string so_path = so_env ? so_env : std::string(TIP_LIB_DIR) + "/libtip_tactile_universa.so";
    mj_loadPluginLibrary(so_path.c_str());

    std::string model_path  = std::string(TIP_REPO_ROOT) + "/samples/tip/scene/universa_finger.xml";
    g_model = LoadModel(model_path, object);
    g_data = mj_makeData(g_model);

    int sensor_id = mj_name2id(g_model, mjOBJ_SENSOR, "finger_tactile");
    int joint_id  = mj_name2id(g_model, mjOBJ_JOINT, "probe_z");
    if (sensor_id < 0 || joint_id < 0) mju_error("模型缺少 finger_tactile 传感器或 probe_z 关节");
    int sensor_adr = g_model->sensor_adr[sensor_id];
    int probe_dof  = g_model->jnt_dofadr[joint_id];

    tip::TactileRos2Publisher ros2_pub(g_model, g_data,
                                          {
                                              { kRos2Topic, "finger_tactile", kRows, kCols },
                                          });

    if (!glfwInit()) mju_error("GLFW 初始化失败");
    g_window = glfwCreateWindow(1000, 800, "universa_finger", nullptr, nullptr);
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

    std::printf("按压对象: %s\n沿法线方向连续加压(关掉 3D 窗口结束)……\n\n", object.c_str());
    std::fflush(stdout);

    int k = 0;
    while (!glfwWindowShouldClose(g_window)) {
        double f                        = AppliedForce(g_data->time);
        g_data->qfrc_applied[probe_dof] = -f; // 沿 -Z 压向垫面
        mj_step(g_model, g_data);
        ++k;
        if (k % kDrawEvery == 0) {
            Render();
            double sensed = mju_sum(g_data->sensordata + sensor_adr, kNumTaxels);
            std::printf("t=%6.2fs  applied=%6.2fN  sensed=%6.2fN\n", g_data->time, f, sensed);
            std::fflush(stdout);
        }
    }

    ros2_pub.Stop();
    mjv_freeScene(&g_scn);
    mjr_freeContext(&g_con);
    glfwDestroyWindow(g_window);
    glfwTerminate();
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    try {
        std::string object = ParseObject(argc, argv);
        return object.empty() ? EXIT_SUCCESS : RunSample(object);
    }
    catch (const std::invalid_argument& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return EXIT_FAILURE;
    }
}
