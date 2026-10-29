#!/usr/bin/env python3
"""探针: 确认 use_sim_time 下 get_clock().now() 拿到的是仿真时间"""
import rclpy
from rclpy.node import Node


class Probe(Node):
    def __init__(self):
        super().__init__('p74_probe', parameter_overrides=[])
        self.get_logger().info('use_sim_time=%s' % self.get_parameter('use_sim_time').value)
        self.create_timer(1.0, self._t)

    def _t(self):
        n = self.get_clock().now()
        self.get_logger().info('clock_ns=%d  (%.3f s)' % (n.nanoseconds, n.nanoseconds * 1e-9))


rclpy.init()
p = Probe()
for _ in range(4):
    rclpy.spin_once(p, timeout_sec=2.0)
p.destroy_node()
rclpy.try_shutdown()
