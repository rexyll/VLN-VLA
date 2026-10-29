#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58e: 规划路径可视化(需求3)。

两条通道:
  1. ROS `nav_msgs/Path` -> `/robot/plan_path`  (rviz/工具可看, 也可被评测
     脚本订阅后画进轨迹图);
  2. Gazebo GUI 里的实体点列: world 里预置了 plan_dot_00..23 共 24 个
     无碰撞小球, 通过 UserCommands 的 set_pose 服务把它们摆到路径上,
     用不到的移到 z=-20 藏起来。

为什么不用 gz.msgs.Marker: gz-sim 8.15 的发行包里没有 marker-manager
系统插件(插件目录下只有 scene-broadcaster / user-commands 等), 发出去
的 Marker 没人渲染。UserCommands 是现成的, 且项目里已经在用它摆人和车。
"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ---------------- A. import ----------------
OLD_A = """from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String"""
assert s.count(OLD_A) == 1
NEW_A = """from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String"""
s = s.replace(OLD_A, NEW_A, 1)

# ---------------- B. 常量 ----------------
OLD_B = "ROBOT_R = 0.18          # 机器人外接半径"
assert s.count(OLD_B) == 1
NEW_B = """ROBOT_R = 0.18          # 机器人外接半径
GZ_WORLD = 'follow_world'
# P58: world 里预置的路径可视化点(无碰撞小球), 见 work_area.sdf
PLAN_DOTS = 24"""
s = s.replace(OLD_B, NEW_B, 1)

# ---------------- C. __init__ ----------------
OLD_C = """        self._plan_dbg = ''           # 当前规划摘要(供 status)"""
assert s.count(OLD_C) == 1
NEW_C = """        self._plan_dbg = ''           # 当前规划摘要(供 status)
        # P58 需求3: 规划路径可视化
        self._path_pub = self.create_publisher(Path, '/robot/plan_path', 10)
        self._vis_plan_t = 0.0
        self._dots_ver = -1           # 已绘制到 gz 的规划版本号
        self._dots_ok = True
        self._dots_busy = False"""
s = s.replace(OLD_C, NEW_C, 1)

# ---------------- D. 发布方法 ----------------
OLD_D = "    def _log(self, now, src, dist, pspd, rspd, w_t, brg, note, nscan,\n"
assert s.count(OLD_D) == 1
NEW_D = '''    # ---------- 规划路径可视化(P58, 需求3) ----------
    @staticmethod
    def _resample(pts, n):
        """沿折线等距取 n 个点(含首尾)。"""
        if not pts:
            return []
        if len(pts) == 1:
            return [pts[0]] * n
        segs = []
        total = 0.0
        for a, b in zip(pts, pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            segs.append((a, b, L))
            total += L
        if total < 1e-6:
            return [pts[0]] * n
        out = []
        k = 0
        acc = 0.0
        for i in range(n):
            s_t = total * i / max(1, n - 1)
            while k < len(segs) - 1 and acc + segs[k][2] < s_t:
                acc += segs[k][2]
                k += 1
            a, b, L = segs[k]
            t = 0.0 if L < 1e-9 else max(0.0, min(1.0, (s_t - acc) / L))
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t))
        return out

    def _publish_plan_vis(self, now):
        _pl = self._planner
        pts = list(_pl.pts)
        if len(pts) < 2:
            return
        # --- 通道 1: nav_msgs/Path ---
        if now - self._vis_plan_t > 0.20:
            self._vis_plan_t = now
            try:
                m = Path()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                for (x, y) in pts:
                    ps = PoseStamped()
                    ps.header = m.header
                    ps.pose.position.x = float(x)
                    ps.pose.position.y = float(y)
                    ps.pose.orientation.w = 1.0
                    m.poses.append(ps)
                self._path_pub.publish(m)
            except Exception:                       # noqa: BLE001
                pass
        # --- 通道 2: Gazebo 里的点列。只在**重规划**时重绘 ---
        if not self._dots_ok or self._dots_busy:
            return
        if _pl.version == self._dots_ver:
            return
        self._dots_ver = _pl.version
        self._dots_busy = True
        th = threading.Thread(target=self._send_dots,
                              args=(self._resample(pts, PLAN_DOTS),),
                              daemon=True)
        th.start()

    def _send_dots(self, pts):
        try:
            from gz.msgs10.pose_pb2 import Pose as GzPose
            from gz.msgs10.boolean_pb2 import Boolean as GzBool
        except Exception as e:                      # noqa: BLE001
            self._dots_ok = False
            self.get_logger().warn(
                'FOLLOWER 规划路径无法在 Gazebo 中绘制(%s), 仍发布到 '
                '/robot/plan_path' % e)
            return
        ok_n = 0
        for i in range(PLAN_DOTS):
            req = GzPose()
            req.name = 'plan_dot_%02d' % i
            req.orientation.w = 1.0
            if i < len(pts):
                req.position.x = float(pts[i][0])
                req.position.y = float(pts[i][1])
                req.position.z = 0.055
            else:
                req.position.z = -20.0          # 多余的点藏到地下
            try:
                r = self._gz_node.request(
                    '/world/%s/set_pose' % GZ_WORLD, req, GzPose, GzBool, 400)
                if isinstance(r, tuple):
                    ok_n += 1 if r[0] else 0
                elif r:
                    ok_n += 1
            except Exception:                       # noqa: BLE001
                pass
        if ok_n == 0:
            self._dots_ok = False
            self.get_logger().warn(
                'FOLLOWER world 中没有 plan_dot_* 模型, Gazebo 看不到路径线'
                '(ROS 侧 /robot/plan_path 仍正常)')
        self._dots_busy = False

    def _log(self, now, src, dist, pspd, rspd, w_t, brg, note, nscan,
'''
s = s.replace(OLD_D, NEW_D, 1)

# ---------------- E. 每个出口都调用 ----------------
OLD_E = """        self._publish(self.v * float(self.cfg['cmd_gain']), w_t)
        self._last_w = w_t"""
assert s.count(OLD_E) == 1
NEW_E = """        self._publish(self.v * float(self.cfg['cmd_gain']), w_t)
        self._last_w = w_t
        if self.cfg['use_planner']:
            self._publish_plan_vis(now)"""
s = s.replace(OLD_E, NEW_E, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('e ok')
