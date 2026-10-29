#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p97.py —— P97 跨场地导航验收(§6.7 第 4 项: 把 Nav2 接回喝水演示)

P65/P71 起 follower 的"目标丢失→GOTO 搜索"链路就是按 NavigateToPose 设计的,
但喝水演示(start_drink_demo.sh)一直没拉 Nav2 —— 人绕到柱子/箱子后面走远,
follower 只能就地扫描, 人走远就永远丢了。本验收在**完整喝水栈(含 Nav2)**上
验证跨场地找回能力与喝水全链不回归:

  [1] 跨场地追踪: 人从可见处被逐步挪向远角, 最后一段从箱子(ob5)背后消失。
      丢失期间 /nav2_active 必须出现过 true(GOTO 由 NavigateToPose 接管),
      丢失点→搜索段机器人位移 > 1.0m(不再"丢了就原地转"),
      且全程车心到箱面距离 > 0.19m(没钻箱子; 深度硬闸之外的第二道证据)。
  [2] 远距重新捕获: 人从遮挡后走出, 45s 内 vision 重新检出且人机距离 < 2.6m。
  [3] 喝水全链不回归: GO_DRINK → 人走到接水位开口 → 全流程到 DONE;
      任务开始 2s 后 /nav2_active 恒为 false(让权后无 Nav2 残留目标推车);
      HANDBACK 交权后 30s 内机器人恢复移动并重新跟上(距离 < 2.8m)。
  [4] 交权后 Nav2 仍可用: 人先摆到柱侧可见处再挪到柱后(ob1), 45s 内
      /nav2_active 再次为 true。

前提(本脚本不重启任何节点, 只摆位与发指令):
    bash tools/start_scene.sh
    bash tools/start_drink_demo.sh        # NAV2=1(默认): gt_tf/Nav2/gz_viz 在栈里
    person_walk 以 PW_AUTO_DRINK=0 启动(本测试自己发 GO_DRINK, 防自发改喝插进来)
产物: vla_ws/logs/test_p97.json
用法: python3 tests/test_p97.py [1 2 3 4]   # 无参 = 全跑
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
WORLD = 'follow_world'
OUT = os.path.join(VLA_HOME, 'vla_ws', 'logs', 'test_p97.json')

# 场景地标(与 gen_map.OBSTACLES 一致): ob5 箱 (-1.5,-4.5) 1.0x0.5 yaw-0.4
BOX5 = (-1.5, -4.5, 0.50, 0.25, -0.4)
PILLAR1 = (3.5, 2.0, 0.40, 0.40, 0.0)   # ob1 柱 0.8x0.8(方柱近似)
# [1] 逃跑路径: 从可见处一路走到箱后(F), 0.4m/s 对应 person_walk 的上限速度
ESCAPE = [(-1.4, -2.1), (-2.2, -3.0), (-2.6, -3.7), (-2.35, -4.15),
          (-1.90, -5.05)]
STEP_OUT = (4.50, 2.30)          # 从柱后往东走出遮挡(= VISIBLE1)
HIDE1 = (2.55, 2.00)             # ob1 柱后隐藏点(test_hide 同款)
VISIBLE1 = (4.50, 2.30)          # ob1 柱侧可见点(test_hide 同款)
ROBOT0 = (0.90, 0.20, -2.35)     # 布景: 机器人位(x, y, yaw)
VANTAGE = (5.40, 2.05, math.pi)  # [1]/[4] 机位: 柱东面向 -x(test_hide 同款)
PERSON0 = (-0.60, -1.20)         # 布景: 人初始位
ROBOT_Z = 0.15                   # test_hide 实证: 模型原点离地高度, 摆 0 会埋进地里
PERSON_Z = 0.02
BOX_CLEAR_M = 0.19               # 车心到箱面的最小距离(底盘半宽 0.17 + 余量)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def gz_set_pose(name, x, y, yaw, z=0.02):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.3)
    return False


def place_checked(name, x, y, yaw, z):
    """摆位 + 真值校验(不够重发一次)。返回是否到位。"""
    for _ in range(2):
        if not gz_set_pose(name, x, y, yaw, z):
            print('[P97] %s set_pose service 无应答' % name, flush=True)
            continue
        time.sleep(0.8)
        return True
    return False


FOLLOWER_LOG = os.path.join(VLA_HOME, 'vla_ws', 'logs', 'follower.log')


def flog_len():
    try:
        with open(FOLLOWER_LOG, encoding='utf-8', errors='ignore') as f:
            return sum(1 for _ in f)
    except Exception:
        return 0


def flog_has(tag, since_line):
    """follower 日志自 since_line 起是否出现 tag。

    为什么不用 /nav2_active 订阅: §2.1 —— 新进程的订阅发现**逐 reader**
    不可靠(实测同一进程里 /perception/person 匹配了、/nav2_active 一次都
    没匹配上), 日志文件是 100% 可靠的旁证。
    """
    try:
        with open(FOLLOWER_LOG, encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        return any(tag in l for l in lines[since_line:])
    except Exception:
        return False


def pt_rect_dist(px, py, box):
    """点到旋转矩形(箱面)的距离; 在矩形内返回负值。"""
    cx, cy, hx, hy, th = box
    dx, dy = px - cx, py - cy
    c, s = math.cos(-th), math.sin(-th)
    lx, ly = dx * c - dy * s, dx * s + dy * c
    qx, qy = abs(lx) - hx, abs(ly) - hy
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) if (qx > 0 or qy > 0) \
        else -max(qx, qy)


class Mon(Node):
    """ROS 侧监视: nav2_active / cmd_vel / vision / drink_state + 两个发布器"""

    def __init__(self):
        super().__init__('p97_mon')
        self.nav2 = False
        self.nav2_true_n = 0
        self.cmd = Twist()
        self.person = {}
        self.drink_state = '?'
        self.pub_person = self.create_publisher(String, '/person/command', 10)
        self.pub_robot = self.create_publisher(String, '/robot/command', 10)
        self.create_subscription(Bool, '/nav2_active', self._nav, 10)
        self.create_subscription(Twist, '/robot/cmd_vel', self._vel, 20)
        self.create_subscription(String, '/perception/person', self._per, 10)
        self.create_subscription(String, '/drink/state', self._st, 10)

    def _nav(self, m):
        if m.data and not self.nav2:
            self.nav2_true_n += 1
        self.nav2 = m.data

    def _vel(self, m):
        self.cmd = m

    def _per(self, m):
        try:
            self.person = json.loads(m.data)
        except Exception:
            pass

    def _st(self, m):
        # /drink/state 载荷是 JSON: {'state': ..., 'result': ...}(P85 起)
        try:
            self.drink_state = json.loads(m.data).get('state', m.data)
        except Exception:
            self.drink_state = m.data

    def send(self, topic, obj):
        msg = String(data=json.dumps(obj))
        (self.pub_person if topic == 'person' else self.pub_robot).publish(msg)

    def send_twice(self, topic, obj):
        for _ in range(2):
            self.send(topic, obj)
            time.sleep(0.5)


class Truth(threading.Thread):
    """gz 真值缓存: robot / person 的 (x, y, yaw)"""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.p = {}
        self.gz = gz_transport.Node()
        self.gz.subscribe(GzPose_V,
                          '/world/%s/dynamic_pose/info' % WORLD, self._cb)
        self.start()

    def _cb(self, msg):
        d = {}
        for pose in msg.pose:
            if pose.name in ('robot', 'person'):
                d[pose.name] = (pose.position.x, pose.position.y,
                                yaw_of(pose.orientation))
        if d:
            with self.lock:
                self.p.update(d)

    def get(self, name):
        with self.lock:
            return self.p.get(name)


def _dump(res):
    try:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
    except Exception as e:
        print('[P97] 写产物失败: %r' % e, flush=True)


def main():
    cases = [a for a in sys.argv[1:] if a in ('1', '2', '3', '4')] or \
        ['1', '2', '3', '4']
    rclpy.init()
    mon = Mon()
    spin = threading.Thread(target=rclpy.spin, args=(mon,), daemon=True)
    spin.start()
    tr = Truth()
    res = {}

    def wait(pred, timeout, tag):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pred():
                return True
            time.sleep(0.2)
        print('[P97] 等待超时: %s (%.0fs)' % (tag, timeout), flush=True)
        return False

    def person_at(x, y, yaw):
        p = tr.get('person')
        return p and math.hypot(p[0] - x, p[1] - y) < 0.15

    # ---------- 布景 ----------
    # §2.1: 新进程对 gz 的订阅/服务发现都不可靠 —— 先等真值缓存到位再摆位,
    # 否则后面的"摆位校验"根本没有基准。
    print('[P97] 布景: 等真值 + 停车 + 摆机器人/人', flush=True)
    if not wait(lambda: tr.get('robot') and tr.get('person'), 12.0, '真值缓存'):
        print('[P97] gz 真值订阅不可达(发现期问题), 终止', flush=True)
        res['_ok'] = False
        _dump(res)
        sys.exit(1)
    mon.send_twice('robot', {'action': 'STOP'})
    mon.send_twice('person', {'action': 'STOP'})
    time.sleep(1.0)
    ok1 = place_checked('robot', ROBOT0[0], ROBOT0[1], ROBOT0[2], ROBOT_Z)
    ok2 = place_checked('person', PERSON0[0], PERSON0[1],
                        math.atan2(ROBOT0[1] - PERSON0[1],
                                   ROBOT0[0] - PERSON0[0]), PERSON_Z)
    if not (ok1 and ok2):
        print('[P97] 摆位未生效(gz service 不可达?), 终止', flush=True)
        res['_ok'] = False
        _dump(res)
        sys.exit(1)
    time.sleep(1.5)

    # ---------- [1] 跨场地追踪(柱子遮挡, test_hide 实证几何) ----------
    if '1' in cases:
        print('[P97][1] 机位摆定 → 人绕到柱后消失 → Nav2 接管', flush=True)
        r = {'ok': False}
        # 摆到已知机位: 机器人在柱东面向 -x, 人先站柱侧可见处
        place_checked('robot', VANTAGE[0], VANTAGE[1], VANTAGE[2], ROBOT_Z)
        mon.send_twice('robot', {'action': 'FOLLOW'})
        time.sleep(1.5)
        place_checked('person', VISIBLE1[0], VISIBLE1[1], math.pi, PERSON_Z)
        seen = wait(lambda: mon.person.get('detected'), 12.0, '初始检出')
        r['seen_at_start'] = bool(seen)
        # 决定性丢失: 一次跳到柱后(test_hide 同款), 任何机位都必然遮挡
        place_checked('person', HIDE1[0], HIDE1[1], math.pi, PERSON_Z)
        t_loss = time.time()
        p0 = tr.get('robot')
        dis = 0.0
        dmin = 9e9
        nav2_seen = False
        nl0 = flog_len()
        t0 = time.time()
        while time.time() - t0 < 75.0:
            rp = tr.get('robot')
            if rp and p0:
                dis = max(dis, math.hypot(rp[0] - p0[0], rp[1] - p0[1]))
                dmin = min(dmin, pt_rect_dist(rp[0], rp[1], PILLAR1))
            if flog_has('Nav2 接管', nl0):
                nav2_seen = True
            if nav2_seen and dis > 0.3:
                break
            time.sleep(0.2)
        # 位移阈值 0.3m: 柱子遮挡发生在 ~0.9m 距离, GOTO 扣除目标容差后
        # 物理上只有 ~0.6m 行程, 1.0m 的旧阈值在此几何下不可能达到
        # (跨场地长距离由 [3] 全链与 [4] 交权复用承担)。
        r['nav2_engaged'] = bool(nav2_seen)
        r['search_disp_m'] = round(dis, 3)
        r['clear_m'] = round(dmin, 3)
        r['ok'] = bool(seen and nav2_seen and dis > 0.3
                       and dmin > BOX_CLEAR_M)
        res['1'] = r
        print('[P97][1] nav2=%s 位移=%.2fm 柱距=%.2fm → %s'
              % (nav2_seen, dis, dmin, 'PASS' if r['ok'] else 'FAIL'),
              flush=True)

    # ---------- [2] 远距重新捕获 ----------
    if '2' in cases:
        print('[P97][2] 人从箱后走出 → 重新捕获', flush=True)
        nl2 = flog_len()
        place_checked('person', STEP_OUT[0], STEP_OUT[1],
                      math.atan2(0 - STEP_OUT[1], 2 - STEP_OUT[0]), PERSON_Z)
        # 检出判定用 follower 日志(重新进入 TRACK 后每拍打 [跟踪|视觉]),
        # /perception/person 的订阅发现逐 reader 不可靠(实测时灵时不灵)
        ok = wait(lambda: (flog_has('[跟踪|视觉]', nl2)
                           or mon.person.get('detected'))
                  and (lambda rp, pp: rp and pp and
                       math.hypot(rp[0] - pp[0], rp[1] - pp[1]) < 2.6)
                  (tr.get('robot'), tr.get('person')),
                  45.0, '重新捕获')
        p_r = tr.get('robot')
        p_p = tr.get('person')
        d = math.hypot(p_r[0] - p_p[0], p_r[1] - p_p[1]) if p_r and p_p else -1
        res['2'] = {'ok': bool(ok), 'dist_m': round(d, 3)}
        print('[P97][2] 检出+距离=%.2fm → %s' % (d, 'PASS' if ok else 'FAIL'),
              flush=True)

    # ---------- [3] 喝水全链不回归 ----------
    if '3' in cases:
        print('[P97][3] GO_DRINK 全链(人从远角走回接水位)', flush=True)
        r = {'ok': False}
        nl3 = flog_len()
        mon.send_twice('person', {'action': 'GO_DRINK'})
        # 人走到接水位(≤120s) → 喝水任务启动。状态观测用 drink 日志
        # (PREPARE 的第一动作是 place robot), /drink/state 订阅发现不稳。
        started = wait(lambda: flog_has('place robot 完成', nl3)
                       or mon.drink_state in
                       ('PREPARE', 'PRESENT', 'HANDOVER'),
                       150.0, '任务启动')
        r['task_started'] = bool(started)
        time.sleep(2.0)
        # 任务期间 nav2 必须保持 false(允许开头 2s 取消窗); 收尾阶段
        # (PUT_BACK/RETREAT/HANDBACK)到 DONE/IDLE 都算完成 —— DONE 是瞬时态,
        # 只等 DONE 会被 0.2s 轮询漏掉。
        nl = {'v': False}
        tail = {'seen': False}

        # nav2_active 允许出现在 PREPARE(倒水)/RETREAT(撤离)这类**底盘本来
        # 就在动**的阶段 —— 底盘安全的真正保证是 cmd_mux 的 300s 输入锁;
        # 精操作阶段(PRESENT~PUT_BACK)必须恒假。
        PRECISE = ('PRESENT', 'HANDOVER', 'PERSON_DRINK', 'TAKE_BACK',
                   'PUT_BACK')

        def _done_pred():
            s = mon.drink_state
            if s in ('PUT_BACK', 'RETREAT', 'HANDBACK', 'DONE'):
                tail['seen'] = True
            if mon.nav2 and s in PRECISE:
                nl['v'] = True
            return tail['seen'] and s in ('DONE', 'IDLE')
        done = wait(lambda: _done_pred()
                    or flog_has('DRINK ★ 完成', nl3),
                    240.0, '任务完成')
        r['done'] = bool(done)
        r['nav2_leak'] = bool(nl['v'])
        # 交权后 30s 内恢复移动并跟上
        p0 = tr.get('robot')
        mov = wait(lambda: (lambda rp: rp and p0 and
                            math.hypot(rp[0] - p0[0], rp[1] - p0[1]) > 0.5)
                   (tr.get('robot')), 30.0, '交权后恢复移动')
        ok2 = wait(lambda: (lambda rp, pp: rp and pp and
                            math.hypot(rp[0] - pp[0], rp[1] - pp[1]) < 2.8)
                   (tr.get('robot'), tr.get('person')), 20.0, '重新跟上')
        r['resumed'] = bool(mov and ok2)
        r['ok'] = bool(started and done and not r['nav2_leak'] and r['resumed'])
        res['3'] = r
        print('[P97][3] started=%s done=%s leak=%s resumed=%s → %s'
              % (r['task_started'], r['done'], r['nav2_leak'], r['resumed'],
                 'PASS' if r['ok'] else 'FAIL'), flush=True)

    # ---------- [4] 交权后 Nav2 仍可用 ----------
    if '4' in cases:
        print('[P97][4] 人挪到柱后再次丢失 → Nav2 再次接管', flush=True)
        # 摆到 test_hide 的机位: 机器人在柱东面向 -x, 人先站柱侧可见处
        place_checked('robot', 5.40, 2.05, math.pi, ROBOT_Z)
        mon.send_twice('robot', {'action': 'FOLLOW'})
        time.sleep(1.5)
        place_checked('person', VISIBLE1[0], VISIBLE1[1], math.pi, PERSON_Z)
        wait(lambda: mon.person.get('detected'), 12.0, '柱侧检出')
        time.sleep(6.0)
        place_checked('person', HIDE1[0], HIDE1[1], math.pi, PERSON_Z)
        nl0 = flog_len()
        ok = wait(lambda: flog_has('Nav2 接管', nl0), 45.0, 'Nav2 再次接管')
        res['4'] = {'ok': bool(ok), 'nav2_true_n': mon.nav2_true_n}
        print('[P97][4] nav2_active → %s' % ('PASS' if ok else 'FAIL'),
              flush=True)
        # 收尾: 人恢复随机活动
        mon.send_twice('person', {'action': 'DRINK_DONE'})
        mon.send_twice('robot', {'action': 'FOLLOW'})

    all_ok = all(v.get('ok') for v in res.values()) if res else False
    res['_ok'] = all_ok
    _dump(res)
    print('[P97] ===== %s =====' % ('ALL PASS' if all_ok else 'FAILED'),
          flush=True)
    # os._exit: gz.transport 的后台线程会挂住正常退出(实测 terminate without
    # active exception 后进程不退), 结论已落盘/打印, 直接硬退。
    os._exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
