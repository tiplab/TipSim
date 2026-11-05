#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""后台线程定时发布 MuJoCo 触觉数据，采用 ROS2 mono16 图像编码。"""
import threading
import time

import numpy as np
import mujoco
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

FORCE_MAX = 50.0  # 单触点编码上限（N）


class TactileRos2Publisher:
    """topics 每项为 (topic, sensor_name, rows, cols[, prepend])。

    rows * cols 必须等于传感器维度。prepend 为 (sensor_name, rows, cols)，
    该表面居中补零后拼接在主表面上方，保留原有行列顺序。"""

    def __init__(self, model, data, topics, node_name="tactile_bridge", rate_hz=30.0):
        self._data = data
        self._rate_hz = float(rate_hz)
        self._sensors = []
        for entry in topics:
            if len(entry) not in (4, 5):
                raise ValueError("topic 项应为 (topic, sensor_name, rows, cols[, prepend])")
            topic, sensor_name, rows, cols = entry[:4]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            assert sid >= 0, f"找不到传感器 {sensor_name}"
            adr, dim = int(model.sensor_adr[sid]), int(model.sensor_dim[sid])
            assert dim == rows * cols, f"{sensor_name}: dim={dim} != rows*cols={rows * cols}"
            prepend = None
            if len(entry) > 4:
                prepend_name, prepend_rows, prepend_cols = entry[4]
                prepend_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, prepend_name)
                assert prepend_sid >= 0, f"找不到传感器 {prepend_name}"
                prepend_adr = int(model.sensor_adr[prepend_sid])
                prepend_dim = int(model.sensor_dim[prepend_sid])
                assert prepend_rows > 0 and prepend_cols > 0, f"{prepend_name}: 行列数必须为正"
                assert prepend_dim == prepend_rows * prepend_cols, (
                    f"{prepend_name}: dim={prepend_dim} != rows*cols={prepend_rows * prepend_cols}")
                assert prepend_cols <= cols, f"{prepend_name}: 拼接表面宽于主表面"
                prepend = (prepend_adr, prepend_dim, prepend_rows, prepend_cols)
            self._sensors.append((topic, sensor_name, adr, dim, rows, cols, prepend))
        self._node_name = node_name
        self._stop_evt = threading.Event()
        self._ready_evt = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._ready_evt.wait(timeout=5.0)

    def _read_force(self, adr, dim, rows, cols, prepend=None):
        def read_surface(source_adr, source_dim, source_rows, source_cols):
            force = np.asarray(self._data.sensordata[source_adr:source_adr + source_dim],
                               dtype=np.float64).reshape(source_rows, source_cols)
            return force

        force = read_surface(adr, dim, rows, cols)
        if prepend is not None:
            prefix = read_surface(*prepend)
            padding = np.zeros((prefix.shape[0], force.shape[1]), dtype=np.float64)
            offset = (force.shape[1] - prefix.shape[1]) // 2
            padding[:, offset:offset + prefix.shape[1]] = prefix
            force = np.vstack((padding, force))
        return force

    def _run(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node(self._node_name)
        pubs = [(topic, sensor_name, node.create_publisher(Image, topic, 10), adr, dim, rows, cols, prepend)
                for topic, sensor_name, adr, dim, rows, cols, prepend in self._sensors]
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        print(f"[ROS2] {self._node_name}: 触觉{len(pubs)}路 "
              + ", ".join(t for t, *_ in pubs) + f"  {self._rate_hz:g}Hz")
        self._ready_evt.set()

        period = 1.0 / self._rate_hz
        while not self._stop_evt.is_set():
            t0 = time.time()
            stamp = node.get_clock().now().to_msg()
            for topic, sensor_name, pub, adr, dim, rows, cols, prepend in pubs:
                force = self._read_force(adr, dim, rows, cols, prepend)
                out_rows, out_cols = force.shape
                u16 = np.round(np.clip(force, 0.0, FORCE_MAX) * (65535.0 / FORCE_MAX)).astype(np.uint16)
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = topic.rsplit("/", 1)[-1]
                msg.height, msg.width = out_rows, out_cols
                msg.encoding = "mono16"
                msg.is_bigendian = 0
                msg.step = out_cols * 2
                msg.data = u16.tobytes()
                pub.publish(msg)

            # 处理 ROS2 发现及通信事件。
            executor.spin_once(timeout_sec=0.01)
            dt = time.time() - t0
            self._stop_evt.wait(max(0.0, period - dt))

        executor.remove_node(node)
        node.destroy_node()

    def stop(self):
        self._stop_evt.set()
        self._thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()
