// 共用的 ROS2 触觉发布器，按目标系统的接口约定发布触觉 (sensor_msgs/Image, mono16, 力(N)
// 线性映射到 0-65535 满量程); 节点/发布器/executor 的创建和 spin 全部在后台线程自己内部完成,
// 不阻塞/拖慢仿真主线程的物理步进
//
// 同步方式: 不是每个 mj_step 都发一帧, 后台线程按自己的频率周期性读取 mjData.sensordata 当前
// 最新值 -- 这块内存在 mjData 生命周期内是稳定的同一块视图, 不需要每步手动同步。
//
// 用法:
//   TactileRos2Publisher pub(m, d, {
//       {"/inspire/right_hand/tactile_13", "finger_tactile", 10, 8},
//   });
//   ...
//   pub.Stop();
//
// prepend_sensor_name/rows/cols 可指定额外表面: 将额外表面居中补零到主表面的列数,
// 拼接在主表面上方。例: zd(3x3) + zj(12x8) -> 15x8。保持传感器原有行列顺序。
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>
#include <vector>

namespace tip
{
    struct TactileTopic
    {
        std::string topic;
        std::string sensor_name;
        int         rows      = 0;
        int         cols      = 0;
        std::string prepend_sensor_name;
        int         prepend_rows = 0;
        int         prepend_cols = 0;
    };

    class TactileRos2Publisher
    {
    public:
        TactileRos2Publisher(const mjModel *m, const mjData *d, std::vector<TactileTopic> topics,
                             std::string node_name = "tactile_bridge", double rate_hz = 30.0)
            : data_(d), rate_hz_(rate_hz), node_name_(std::move(node_name))
        {
            for (auto& t : topics) {
                int sid = mj_name2id(m, mjOBJ_SENSOR, t.sensor_name.c_str());
                if (sid < 0) mju_error("[TactileRos2Publisher] 找不到传感器 %s", t.sensor_name.c_str());
                int adr = m->sensor_adr[sid];
                int dim = m->sensor_dim[sid];
                if (dim != t.rows * t.cols) {
                    mju_error("[TactileRos2Publisher] %s: dim=%d != rows*cols=%d", t.sensor_name.c_str(),
                              dim, t.rows * t.cols);
                }
                Entry e;
                e.topic     = t.topic;
                e.adr       = adr;
                e.dim       = dim;
                e.rows      = t.rows;
                e.cols      = t.cols;
                if (!t.prepend_sensor_name.empty()) {
                    int prepend_sid = mj_name2id(m, mjOBJ_SENSOR, t.prepend_sensor_name.c_str());
                    if (prepend_sid < 0)
                        mju_error("[TactileRos2Publisher] 找不到传感器 %s", t.prepend_sensor_name.c_str());
                    e.prepend_adr  = m->sensor_adr[prepend_sid];
                    e.prepend_dim  = m->sensor_dim[prepend_sid];
                    e.prepend_rows = t.prepend_rows;
                    e.prepend_cols = t.prepend_cols;
                    if (e.prepend_rows <= 0 || e.prepend_cols <= 0 ||
                        e.prepend_dim != e.prepend_rows * e.prepend_cols)
                        mju_error("[TactileRos2Publisher] %s: dim=%d != rows*cols=%d",
                                  t.prepend_sensor_name.c_str(), e.prepend_dim, e.prepend_rows * e.prepend_cols);
                    if (e.prepend_cols > e.cols)
                        mju_error("[TactileRos2Publisher] %s: 拼接表面宽于主表面",
                                  t.prepend_sensor_name.c_str());
                }
                entries_.push_back(std::move(e));
            }
            thread_ = std::thread(&TactileRos2Publisher::Run, this);
            // 等后台线程把节点/发布器建好(见类注释: 不能跨线程用同一个 rclcpp 对象)。
            for (int i = 0; i < 500 && !ready_.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        ~TactileRos2Publisher() { Stop(); }

        TactileRos2Publisher(const TactileRos2Publisher&)            = delete;
        TactileRos2Publisher& operator=(const TactileRos2Publisher&) = delete;

        void Stop()
        {
            if (stop_.exchange(true)) return; // 已经 stop 过
            if (thread_.joinable()) thread_.join();
            if (rclcpp::ok()) rclcpp::shutdown();
        }

    private:
        struct Entry
        {
            std::string                                           topic;
            int                                                   adr, dim, rows, cols;
            int prepend_adr = -1, prepend_dim = 0, prepend_rows = 0, prepend_cols = 0;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
        };

        // ── 后台线程: 节点/发布器/spin 全部在这里创建和使用 ──
        void Run()
        {
            if (!rclcpp::ok()) rclcpp::init(0, nullptr);
            auto node = rclcpp::Node::make_shared(node_name_);
            for (auto& e : entries_)
                e.pub = node->create_publisher<sensor_msgs::msg::Image>(e.topic, 10);

            std::printf("[ROS2] %s: 触觉%zu路", node_name_.c_str(), entries_.size());
            for (auto& e : entries_)
                std::printf(" %s", e.topic.c_str());
            std::printf("  %gHz\n", rate_hz_);
            std::fflush(stdout);
            ready_ = true;

            const auto            period = std::chrono::duration<double>(1.0 / rate_hz_);
            std::vector<double>   raw;
            std::vector<double>   prepend_raw, combined;
            std::vector<uint16_t> u16;
            constexpr double      kForceMax = 50.0; // N, force -> mono16 满量程, 跟 Python 版一致

            while (!stop_.load()) {
                auto         t0    = std::chrono::steady_clock::now();
                rclcpp::Time stamp = node->get_clock()->now();
                for (auto& e : entries_) {
                    raw.assign(data_->sensordata + e.adr, data_->sensordata + e.adr + e.dim);
                    int rows = e.rows;
                    if (e.prepend_adr >= 0) {
                        prepend_raw.assign(data_->sensordata + e.prepend_adr,
                                           data_->sensordata + e.prepend_adr + e.prepend_dim);
                        combined.assign((e.prepend_rows + rows) * e.cols, 0.0);
                        int offset = (e.cols - e.prepend_cols) / 2;
                        for (int r = 0; r < e.prepend_rows; ++r)
                            std::copy_n(prepend_raw.begin() + r * e.prepend_cols, e.prepend_cols,
                                        combined.begin() + r * e.cols + offset);
                        std::copy(raw.begin(), raw.end(), combined.begin() + e.prepend_rows * e.cols);
                        raw.swap(combined);
                        rows += e.prepend_rows;
                    }
                    u16.resize(raw.size());
                    for (size_t k = 0; k < raw.size(); ++k) {
                        double v = std::min(std::max(raw[k], 0.0), kForceMax);
                        u16[k]   = static_cast<uint16_t>(std::lround(v * (65535.0 / kForceMax)));
                    }

                    sensor_msgs::msg::Image msg;
                    msg.header.stamp    = stamp;
                    auto slash          = e.topic.find_last_of('/');
                    msg.header.frame_id = slash == std::string::npos ? e.topic : e.topic.substr(slash + 1);
                    msg.height          = rows;
                    msg.width           = e.cols;
                    msg.encoding        = "mono16";
                    msg.is_bigendian    = 0;
                    msg.step            = e.cols * 2;
                    msg.data.resize(u16.size() * 2);
                    std::memcpy(msg.data.data(), u16.data(), msg.data.size());
                    e.pub->publish(msg);
                }

                // 纯发布节点如果完全不 spin, discovery/graph 事件处理不到, 发布方和订阅方会
                // 一直配对不上(哪怕 topic 名字/类型都对得上)。
                rclcpp::spin_some(node);
                auto dt = std::chrono::steady_clock::now() - t0;
                auto sleep_for =
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period) - dt;
                if (sleep_for > sleep_for.zero()) std::this_thread::sleep_for(sleep_for);
            }
        }

        const mjData      *data_;
        double             rate_hz_;
        std::string        node_name_;
        std::vector<Entry> entries_;
        std::thread        thread_;
        std::atomic<bool>  stop_{ false };
        std::atomic<bool>  ready_{ false };
    };

} // namespace tip
