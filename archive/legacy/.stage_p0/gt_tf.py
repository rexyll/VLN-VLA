#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gt_tf.py —— Gazebo 真值 → Nav2 定位桥 (P65)

订阅: gz /world/follow_world/dynamic_pose/info
发布: TF map→odom (静态, 1Hz 重发) + odom→base_link (30Hz)
      /odom (nav_msgs/Odometry, 供速度平滑/调试)
说明: GPU 激光在本机渲染栈输出无效(P2), AMCL 不可用 → 用仿真真值定位,
     这是仿真环境的标准做法; 世界系即 map 系(地图原点见 arena.yaml)。

P71 修复: yaw 公式分母原为 1-2(y²+x²), 应为 1-2(y²+z²)。
     R = Rz(ψ)Ry(θ)Rx(φ) 展开后 yaw = atan2(2(wz+xy), 1-2(y²+z²))。
     写错成 x² 后, 平面运动(x=y=0)时分母=1、分子=2wz → atan2 结果正好被
     折半(60° 读成 30°), Nav2 全局规划/局部控制全部走偏, 历史"原地打转"
     根因第 1 条即是此式。
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V


def quat_to_yaw(q):
    """四元数 → 偏航角(绕 z)。分母用 y²+z², 不要写成 y²+x²。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class GtTf(Node):

    def __init__(self):
        super().__init__('gt_tf')
        self.declare_parameter('map_origin_x', -6.15)
        self.declare_parameter('map_origin_y', -6.15)
        self.declare_parameter('map_res', 0.05)
        self.mox = self.get_parameter('map_origin_x').value
        self.moy = self.get_parameter('map_origin_y').value
        self.mres = self.get_parameter('map_res').value
        self.tf_broad = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V,
                           '/world/follow_world/dynamic_pose/info',
                           self._pose_cb)
        self._lock = threading.Lock()
        self.rx = self.ry = 0.0
        self.ryaw = 0.0
        self._got = False
        # map→odom 静态恒等(真值即里程计)
        st = TransformStamped()
        st.header.frame_id = 'map'
        st.child_frame_id = 'odom'
        st.transform.rotation.w = 1.0
        self.tf_static.sendTransform(st)
        self.create_timer(1.0, self._restatic)
        self.get_logger().info(
            'GT_TF 真值定位桥就绪: map→odom→base_link (/odom 同步发布)')

    def _restatic(self):
        st = TransformStamped()
        st.header.frame_id = 'map'
        st.child_frame_id = 'odom'
        st.transform.rotation.w = 1.0
        self.tf_static.sendTransform(st)

    def _pose_cb(self, msg):
        for p in msg.pose:
            if p.name != 'robot':
                continue
            yaw = quat_to_yaw(p.orientation)
            with self._lock:
                self.rx, self.ry, self.ryaw = \
                    p.position.x, p.position.y, yaw
            self._publish()
            return

    def _publish(self):
        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
        now = self.get_clock().now().to_msg()
        # odom→base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(rx)
        t.transform.translation.y = float(ry)
        t.transform.rotation = yaw_quat(ryaw)
        self.tf_broad.sendTransform(t)
        # /odom
        om = Odometry()
        om.header.stamp = now
        om.header.frame_id = 'odom'
        om.child_frame_id = 'base_link'
        om.pose.pose.position.x = float(rx)
        om.pose.pose.position.y = float(ry)
        om.pose.pose.orientation = yaw_quat(ryaw)
        self.pub_odom.publish(om)


def main(args=None):
    rclpy.init(args=args)
    node = GtTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
