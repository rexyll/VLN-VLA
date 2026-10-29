#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""person_walk.py —— "人"的随机行走控制器

行为: 启动 3 秒后开始在 12m x 12m 工作区内随机行走,
     速度在 0.25~0.65 m/s 间随机变化, 随机转向, 偶尔明显减速/停顿;
     靠近墙壁或障碍物时进入 ESCAPE 状态径直走回场地中心,
     长时间没有位移(被卡住)时也强制逃逸。
"""
import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ARENA_HALF = 5.3   # 工作区内边界(墙内侧 6.0m, 留出人体半径)
WALL_MARGIN = 0.35  # 距墙多少米算危险
# (x, y, 危险半径) 与 world 中障碍物一致, 含人体半径余量(收紧以减少逃逸循环)
# P22 修正: 只保留 work_area.sdf 里真实存在的 3 根立柱。
# 原表有 4 项在世界中没有任何几何体, 人一走进这些"幽灵危险区"就转 ESCAPE
# 并原地抖动(实测位置长时间卡在 (+1.00,+0.00) 不动)。
# 半径 = 立柱半径 + 0.2m 人体余量。
# P24 修正: 与 follower 同步为 SDF 里真实存在的 7 个障碍(半径 +0.2m 人体余量)
OBSTACLES = [
    (3.5, 2.0, 0.62),
    (-3.0, 3.2, 0.72),
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.70),
    (4.2, -3.0, 0.62),
    (-1.5, -4.5, 0.76),
    (2.2, 4.6, 0.55),
]


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def in_danger(x, y, margin=0.0):
    """当前位置是否贴墙/撞障碍(margin 为额外余量)"""
    if abs(x) > ARENA_HALF - WALL_MARGIN + margin or \
            abs(y) > ARENA_HALF - WALL_MARGIN + margin:
        return True
    for (ox, oy, r) in OBSTACLES:
        if math.hypot(x - ox, y - oy) < r + margin:
            return True
    return False


class PersonWalk(Node):

    def __init__(self):
        super().__init__('person_walk')
        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 10)
        self._lock = threading.Lock()
        self.x, self.y, self.yaw = 1.0, 0.0, 0.0
        self.rx, self.ry = 0.0, 0.0  # 机器人位置(动态避让)
        self.state = 'WAIT'
        self.state_until = time.monotonic() + 3.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.turn_rate = 0.0
        self._last_pos_log = 0.0
        self._vmod_t = 0.0          # P33: WALK 段内速度抖动计时
        # 逃逸与防卡死
        self._clear_since = None
        self._progress_pos = (1.0, 0.0)
        self._progress_t = time.monotonic()
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info('PERSON "人"节点就绪, 3 秒后开始在工作区内随机行走')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.x, self.y, self.yaw = p.x, p.y, yaw

    def _robot_cb(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self.rx, self.ry = p.x, p.y

    def _pos(self):
        with self._lock:
            return self.x, self.y, self.yaw

    def _pick_walk(self):
        # P33: 速度幅值拉开(原 0.20~0.45 变化不明显, 体现不出"时快时慢")
        # P35: 上限 0.60→0.55 —— 机器人 approach_speed=0.52, 人要能被跟上
        # P36: 0.55→0.50, 给速度环留出真正的跟踪余量
        # P39: 0.50→0.40 —— 对正段只有 0.85*0.52=0.44, 人走 0.50 追不上
        self.target_speed = random.uniform(0.12, 0.40)
        self.turn_rate = 0.0
        self.state = 'WALK'
        self.state_until = time.monotonic() + random.uniform(5.0, 9.0)
        self.get_logger().info(f'PERSON 行走中 v={self.target_speed:.2f}m/s')

    def _next_state(self):
        x, y, _ = self._pos()
        # 前方 1.5s 预判有危险时不选择转向/停顿, 继续直线走
        ax = x + math.cos(self.yaw) * 0.15 * 1.5
        ay = y + math.sin(self.yaw) * 0.15 * 1.5
        if in_danger(ax, ay, 0.2):
            self.state_until = time.monotonic() + 1.0
            return
        r = random.random()
        if self.state == 'WALK':
            if r < 0.55:
                rate = random.uniform(0.3, 0.7) * random.choice([-1, 1])
                angle = random.uniform(0.35, 1.6)
                self.state = 'TURN'
                self.turn_rate = rate
                self.target_speed = 0.15
                self.state_until = time.monotonic() + abs(angle / rate)
                self.get_logger().info(f'PERSON 转向 {math.degrees(angle):.0f}°')
            elif r < 0.78:
                self.state = 'SLOW'
                self.turn_rate = 0.0
                self.target_speed = 0.08
                self.state_until = time.monotonic() + random.uniform(2.0, 4.0)
                self.get_logger().info('PERSON 明显减速(慢走)')
            else:
                self.state = 'PAUSE'
                self.turn_rate = 0.0
                self.target_speed = 0.0
                self.state_until = time.monotonic() + random.uniform(1.5, 3.5)
                self.get_logger().info('PERSON 停顿')
        else:
            self._pick_walk()

    def _tick(self):
        dt = 0.05
        now = time.monotonic()
        x, y, yaw = self._pos()
        with self._lock:
            rx, ry = self.rx, self.ry
        # P22: 原阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
        # 机器人刚走到 1.0m 人就开逃, 机器人永远在追。
        # P33: 0.85 仍然太高 —— 距离环允许下探到 0.9m, 一旦过冲(0.85)
        # 人就主动远离, 等于机器人被"追着跑"的对象反击, 1.0±0.1 永远收敛
        # 不了。改成 0.45m 只在贴身(机器人压上来)时让开, 正常跟随不干预。
        # P38: 0.45 → 0.70 —— 人会主动贴到机器人脸上(实测 d=0.41), 那时
        # 相机已经看不到人体表面, 机器人只能判丢失。合理的人机最小间距
        # 应落在跟随容差带(0.9m)之外。
        robot_close = math.hypot(x - rx, y - ry) < 0.70
        static_danger = in_danger(x, y, 0.1)
        # 卡死检测: 8 秒内位移 < 0.3m 且不在等待/停顿 → 强制逃逸
        if self.state in ('WALK', 'TURN', 'SLOW'):
            moved = math.hypot(x - self._progress_pos[0],
                               y - self._progress_pos[1])
            if now - self._progress_t > 8.0:
                if moved < 0.3:
                    self.state = 'ESCAPE'
                    self._clear_since = None
                    self.get_logger().info('PERSON 疑似卡住, 强制返回场地中心')
                self._progress_pos = (x, y)
                self._progress_t = now
        # 进入危险区/离机器人过近 → ESCAPE
        if self.state not in ('ESCAPE', 'WAIT') and (robot_close or static_danger):
            self.state = 'ESCAPE'
            self._clear_since = None
            self._escape_from_robot = robot_close and not static_danger
            if robot_close and not static_danger:
                self.get_logger().info('PERSON 离机器人过近, 绕开')
            else:
                self.get_logger().info('PERSON 靠近边界/障碍, 返回场地中心')

        if self.state == 'WAIT':
            self._publish(0.0, 0.0)
            if now >= self.state_until:
                self._pick_walk()
            return

        if self.state == 'ESCAPE':
            # 总时长看门狗: 逃逸 12s 仍未清场 → 强制随机行走(防死循环)
            if getattr(self, '_esc_enter', 0.0) == 0.0:
                self._esc_enter = now
            elif now - self._esc_enter > 12.0:
                self.get_logger().info('PERSON 逃逸超时, 强制随机行走')
                self._esc_enter = 0.0
                self._esc_bias = 0.0
                self._esc_t0 = 0.0
                self._clear_since = None
                self._progress_pos = (x, y)
                self._progress_t = now
                self._pick_walk()
                return
            if getattr(self, '_escape_from_robot', False) and not static_danger:
                desired = math.atan2(y - ry, x - rx)  # 远离机器人
            else:
                desired = math.atan2(-y, -x)          # 场地中心
            # 进度看门狗: 逃逸受阻(原地卡住)则随机偏置方向, 防止目标被障碍挡死
            if getattr(self, '_esc_t0', 0.0) == 0.0:
                self._esc_t0 = now
                self._esc_pos = (x, y)
            elif now - self._esc_t0 > 6.0:
                if math.hypot(x - self._esc_pos[0], y - self._esc_pos[1]) < 0.25:
                    self._esc_bias = random.uniform(1.0, 2.2) * random.choice([-1, 1])
                    self.get_logger().info(
                        f'PERSON 逃逸受阻, 方向偏置 {math.degrees(self._esc_bias):.0f}°')
                self._esc_t0 = now
                self._esc_pos = (x, y)
            desired += getattr(self, '_esc_bias', 0.0)
            err = wrap(desired - yaw)
            self.turn_rate = max(-1.2, min(1.2, 2.0 * err))
            # 先原地转对准再走: 避免弧线起步撞上机器人把它推走
            self.target_speed = 0.25 if abs(err) < 0.6 else 0.0
            amax = 0.5 * dt
            ds = self.target_speed - self.speed
            self.speed += max(-amax, min(amax, ds))
            self._publish(self.speed, self.turn_rate)
            clear = (not in_danger(x, y, 0.35)) and \
                    math.hypot(x - rx, y - ry) > 1.4
            if clear:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since > 1.2:
                    self._clear_since = None
                    self._esc_bias = 0.0
                    self._esc_t0 = 0.0
                    self._esc_enter = 0.0
                    self._progress_pos = (x, y)
                    self._progress_t = now
                    self._pick_walk()
            else:
                self._clear_since = None
            if now - self._last_pos_log > 2.0:
                self._last_pos_log = now
                self.get_logger().info(
                    f'PERSON 位置=({x:+.2f}, {y:+.2f}) v={self.speed:.2f} 状态=ESCAPE')
            return

        if now >= self.state_until:
            self._next_state()
        # P33: WALK 段内也做速度抖动, 制造"时快时慢"(原实现每段速度恒定)
        if self.state == 'WALK' and now - self._vmod_t > 2.5:
            self._vmod_t = now
            if random.random() < 0.6:
                self.target_speed = random.uniform(0.12, 0.40)
        amax = 0.35 * dt  # 人的加/减速度, 保证速度变化平滑
        ds = self.target_speed - self.speed
        self.speed += max(-amax, min(amax, ds))
        # 前方 1.5s 预判有危险 → 提前向场地中心转向
        ax = x + math.cos(yaw) * max(self.speed, 0.15) * 1.5
        ay = y + math.sin(yaw) * max(self.speed, 0.15) * 1.5
        w = self.turn_rate
        if in_danger(ax, ay, 0.2) or math.hypot(ax - rx, ay - ry) < 1.3:
            to_center = math.atan2(-y, -x)
            w = max(-1.2, min(1.2, 2.0 * wrap(to_center - yaw)))
            self.speed = min(self.speed, 0.2)
        self._publish(self.speed, w)
        if now - self._last_pos_log > 2.0:
            self._last_pos_log = now
            self.get_logger().info(
                f'PERSON 位置=({x:+.2f}, {y:+.2f}) 朝向={math.degrees(yaw):.0f}° '
                f'v={self.speed:.2f} 状态={self.state}')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    # P56: 支持固定随机种子, 让 A/B 对比跑在**同一条人行走轨迹**上。
    # 原来 random 未播种, 每次评测人的路径都不一样 —— 实测同一版代码
    # 三次评测 in_band 分别是 75.3% / 61.4% / 15.9%, 单次结果没有可比性。
    # 用法: PW_SEED=7 ros2 launch ...
    import os
    _sd = os.environ.get('PW_SEED', '').strip()
    if _sd:
        _seed = int(_sd)
        random.seed(_seed)
        print('[PERSON] 随机种子 = %d' % _seed, flush=True)
    rclpy.init(args=args)
    node = PersonWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
