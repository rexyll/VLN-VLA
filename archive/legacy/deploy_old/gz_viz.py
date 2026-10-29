#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_viz.py —— 在 Gazebo 里画「实际轨迹」与「Nav2 规划路径」

为什么不用 marker: 本机 gz-sim 8 的插件目录里没有 marker-manager
(`ls .../gz-sim-8/plugins | grep marker` = 0), 也没有 `/marker` 服务,
所以 ROS 的 visualization_msgs/Marker 传不进 Gazebo。

改用「预创建静态小球 + set_pose 移动」:
  /world/follow_world/create     用 EntityFactory 建一批 static 小球
  /world/follow_world/set_pose   每帧改位姿
静态模型不进 dynamic_pose/info, 所以不污染 gt_tf 的真值链路(它只订阅 dynamic)。

颜色约定:
  蓝色小球  —— 机器人在走      (|cmd_v| >= 0.02)
  红色小球  —— 机器人停住/被限速 (|cmd_v| < 0.02)
  品红小球  —— Nav2 规划路径 (/plan)
蓝色/红色轨迹点合起来就是"走过的路", 一眼能看出在哪段来回摆、在哪段被卡住。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_pb2 import Pose
    _GZ_OK = True
except Exception as _e:                                   # pragma: no cover
    _GZ_OK = False
    _GZ_ERR = repr(_e)

WORLD = '/world/follow_world'
HIDE_Z = -5.0

N_TRAIL = 40          # 蓝色(行进中)轨迹点
N_STOP = 40           # 红色(停住)轨迹点
N_PLAN = 30           # 规划路径点
TRAIL_STEP = 0.22     # 每 0.22m 落一个轨迹点
PLAN_STEP = 0.30      # 规划路径每 0.30m 一个点
DOT_R = 0.085   # P76c: 俯视 13m 全景下 0.045 仍偏小(约 4px), 再加到 8px 左右
                #       相机被锁在 13m 全景(禁掉 CameraTracking), 尺寸得按那个
                #       比例来定; 小球是 static 模型, 不参与碰撞也不进真值链路。


def _sdf(name, r, rgb):
    return (
        '<?xml version="1.0"?><sdf version="1.9">'
        '<model name="%s"><static>true</static>'
        '<pose>0 0 %.1f 0 0 0</pose>'
        '<link name="l"><visual name="v">'
        '<geometry><sphere><radius>%.3f</radius></sphere></geometry>'
        '<material><ambient>%s 1</ambient><diffuse>%s 1</diffuse>'
        '<emissive>%s 1</emissive></material>'
        '<cast_shadows>false</cast_shadows>'
        '</visual></link></model></sdf>' % (name, HIDE_Z, r, rgb, rgb, rgb))


class GzViz(Node):

    def __init__(self):
        super().__init__('gz_viz')
        self.ok = _GZ_OK
        self.gz = None
        if not self.ok:
            self.get_logger().error('gz python 绑定不可用: %s' % _GZ_ERR)
            return
        try:
            self.gz = gz_transport.Node()
        except Exception as e:
            self.get_logger().error('gz transport 初始化失败: %r' % e)
            self.ok = False
            return

        self.names = []
        specs = []
        for i in range(N_TRAIL):
            specs.append(('trk_b_%02d' % i, DOT_R, '0.15 0.45 1.0'))
        for i in range(N_STOP):
            specs.append(('trk_r_%02d' % i, DOT_R, '1.0 0.10 0.10'))
        for i in range(N_PLAN):
            specs.append(('pln_%02d' % i, DOT_R * 0.9, '0.85 0.1 0.9'))
        bad = 0
        for name, r, rgb in specs:
            if self._create(name, r, rgb):
                self.names.append(name)
            else:
                bad += 1
        self.get_logger().info(
            'VIZ 已创建 %d/%d 个可视化小球%s'
            % (len(self.names), len(specs), (', 失败 %d' % bad) if bad else ''))
        if bad == len(specs):
            self.ok = False
            return

        self.rx = self.ry = self.ryaw = 0.0
        self.have_pose = False
        self.cmd_v = 0.0
        self.last_trail = None      # (x, y)
        self.trk_i = 0
        self.stp_i = 0
        self.plan_valid = 0
        self._pending_plan = None

        self.create_subscription(Odometry, '/robot/odom', self._odom_cb, 20)
        self.create_subscription(Twist, '/robot/cmd_vel', self._cmd_cb, 20)
        self.create_subscription(Path, '/plan', self._plan_cb, 5)
        self.create_timer(0.10, self._tick)
        self.get_logger().info('VIZ 就绪: 蓝=行进 红=停住 品红=Nav2 规划路径')

    # ---------------- gz 服务 ----------------
    def _call(self, svc, req, rqt, timeout=800):
        try:
            r = self.gz.request(WORLD + svc, req, rqt, Boolean, timeout)
            return bool(r[0]) and bool(getattr(r[1], 'data', False))
        except Exception:
            return False

    def _create(self, name, r, rgb):
        ef = EntityFactory()
        ef.sdf = _sdf(name, r, rgb)
        ef.name = name
        return self._call('/create', ef, EntityFactory)

    def _move(self, name, x, y, z):
        p = Pose()
        p.name = name
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        return self._call('/set_pose', p, Pose, timeout=300)

    # ---------------- ROS 回调 ----------------
    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rx, self.ry = p.x, p.y
        self.have_pose = True

    def _cmd_cb(self, msg):
        self.cmd_v = msg.linear.x

    def _plan_cb(self, msg):
        pts = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        if pts:
            self._pending_plan = pts

    # ---------------- 主循环 ----------------
    def _tick(self):
        if not self.ok:
            return
        if self._pending_plan:
            self._draw_plan(self._pending_plan)
            self._pending_plan = None
        if not self.have_pose:
            return
        if self.last_trail is None:
            self.last_trail = (self.rx, self.ry)
            return
        d = math.hypot(self.rx - self.last_trail[0],
                       self.ry - self.last_trail[1])
        if d < TRAIL_STEP:
            return
        self.last_trail = (self.rx, self.ry)
        if abs(self.cmd_v) < 0.02:
            name = 'trk_r_%02d' % self.stp_i
            self.stp_i = (self.stp_i + 1) % N_STOP
        else:
            name = 'trk_b_%02d' % self.trk_i
            self.trk_i = (self.trk_i + 1) % N_TRAIL
        self._move(name, self.rx, self.ry, 0.03)

    def _draw_plan(self, pts):
        """把规划路径按 PLAN_STEP 重采样到 N_PLAN 个小球上。"""
        out = []
        acc = 0.0
        prev = pts[0]
        out.append(prev)
        for p in pts[1:]:
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
            if acc >= PLAN_STEP:
                out.append(p)
                acc = 0.0
            if len(out) >= N_PLAN:
                break
        for i in range(N_PLAN):
            if i < len(out):
                self._move('pln_%02d' % i, out[i][0], out[i][1], 0.06)
            else:
                self._move('pln_%02d' % i, 0.0, 0.0, HIDE_Z)
        self.plan_valid += 1
        if self.plan_valid <= 3 or self.plan_valid % 20 == 0:
            self.get_logger().info(
                'VIZ 规划路径 %d 点 (第 %d 次更新)' % (len(out), self.plan_valid))


def main(args=None):
    rclpy.init(args=args)
    n = GzViz()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
