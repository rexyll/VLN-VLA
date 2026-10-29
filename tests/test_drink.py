#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_drink.py —— P91 端到端验收: 一条命令跑完整条"喝水"链

验的是**整条链路**, 不是某一段动作 ——
    send_person_cmd.sh drink
      → 人走过去(PERSON_SPOT) → 站定面向机器人 → 说"我要喝水"
      → voice_node 解析 DRINK → follower/cmd_mux 让权
      → 机器人倒水 → 双手捧杯转身 → 递到人的手 → 人**双手张开迎 → 合拢捧住**
      → 人喝 → **双手张开交还** → 机器人双手接回 → 转回桌前 → 放回原位
      → (P93) 机器人撤离桌边 → 交还底盘给 follower → 人边走机器人边跟上

先起场景和节点:
    bash tools/start_scene.sh
    bash tools/start_drink_demo.sh
    python3 tests/test_drink.py

产物: vla_ws/logs/p91_{arrive,present,drink,takeback,placed}.png
      vla_ws/logs/test_drink.json

**本脚本只读**: 用 `DrinkTask(observer_only=True)` —— 它照旧订阅关节/真值,
但**不挂 20Hz/50Hz 定时器**, 所以 39 路指令和抓取位姿一帧都不发。否则会和
常驻的 drink_task 抢同一个发布者, 杯子被两个目标拽来拽去。
(另: 在本进程里自己 new 一个 `gz.transport13.Node` 会 segfault —— 与 rclpy
初始化混用时的老问题, 所以统一走 DrinkTask 里那套已验证的订阅。)
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import drink_motion as M      # noqa: E402
from vla_follow import drink_poses as P       # noqa: E402
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402
from vla_follow.pngsnap import save_png       # noqa: E402

LOG_DIR = os.path.join(ROOT, 'vla_ws', 'logs')
TOTAL_TIMEOUT = 420.0
# 进入该阶段时抓一张图
# P96: HANDOVER 是"人双手合拢捧住杯子"的那一瞬(合拢动作就排在进
# HANDOVER 之前), 值得单独留一张 —— 这是双手接杯唯一的可见证据。
SHOT_AT = {'PRESENT': 'present', 'HANDOVER': 'grip',
           'PERSON_DRINK': 'drink',
           'TAKE_BACK': 'takeback', 'DONE': 'placed',
           'HANDBACK': 'handback'}


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class Watch(object):
    """把"状态机 + 相机 + 发指令"这三件事挂到一个只读 DrinkTask 上。"""

    def __init__(self, node):
        self.node = node
        self.state = 'BOOT'
        self.stages = []
        self.cmds = []
        self.hands = {}
        self.img = None
        self.t0 = time.time()
        # 仿真时钟(见 _clock_cb): "全流程用时"要用逻辑时长, 不能跟机器快慢走
        self.sim = None
        node.create_subscription(Clock, '/clock', self._clock_cb, 10)
        node.create_subscription(String, '/drink/state', self._state_cb, 10)
        node.create_subscription(String, '/robot/command', self._cmd_cb, 10)
        node.create_subscription(Image, '/drink_cam/image', self._img_cb, 5)
        self.pub = node.create_publisher(String, '/person/command', 10)

    def _clock_cb(self, m):
        self.sim = float(m.clock.sec) + float(m.clock.nanosec) * 1e-9

    def _state_cb(self, m):
        try:
            d = json.loads(m.data)
            s = d.get('state')
        except Exception:
            return
        # P96: HANDOVER 那条会带上"双手开合"的真值测量结果, 存下来给判据用。
        # 这些数是 drink_task 拿**手 link 的真实位姿**算的, 不是回读指令。
        h = d.get('result', {}).get('hands')
        if h:
            self.hands = h
        if s and s != self.state:
            self.state = s
            self.stages.append(s)
            print('    [阶段] %-12s t=%.1fs' % (s, time.time() - self.t0))

    def _cmd_cb(self, m):
        try:
            c = json.loads(m.data)
        except Exception:
            return
        if c.get('action') == 'DRINK':
            self.cmds.append(c)
            print('    [收到 DRINK] t=%.2fs 共 %d 条'
                  % (time.time() - self.t0, len(self.cmds)))

    def _img_cb(self, m):
        self.img = m

    def spin(self, s):
        self.node.spin_for(s)

    def reset(self):
        """清掉"上一轮任务"的残留状态(DONE 心跳/旧阶段), 从 IDLE 重新计数。"""
        self.state = 'IDLE'
        self.stages = []
        self.cmds = []
        self.hands = {}

    def send(self, action):
        self.pub.publish(String(data=json.dumps({'action': action})))
        self.spin(0.6)
        self.pub.publish(String(data=json.dumps({'action': action})))

    def shot(self, tag):
        if self.img is None:
            return None
        path = os.path.join(LOG_DIR, 'p91_%s.png' % tag)
        save_png(path, self.img.data, self.img.width, self.img.height,
                 self.img.encoding)
        return path


def main():
    rclpy.init()
    node = DrinkTask(observer_only=True)
    w = Watch(node)
    fails = []
    res = {'ok': False, 'fails': fails, 'shots': []}

    def step(tag, ok, detail=''):
        print('%s %-52s %s%s' % ('[OK]' if ok else '[NG]', tag,
                                 'OK' if ok else 'NG',
                                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    def pos(name):
        return M.object_pose(node, name)

    def xy(name):
        T = pos(name)
        return None if T is None else (float(T[0, 3]), float(T[1, 3]))

    def yaw(name):
        T = pos(name)
        return None if T is None else math.atan2(T[1, 0], T[0, 0])

    print('=' * 92)
    print('P91 端到端验收 —— 人走过去说"我要喝水" → 倒水递水 → 人喝 → 接回放回')
    print('=' * 92)

    # ---- [1] 数据链路 + 起始状态
    t0 = time.time()
    while time.time() - t0 < 20.0 and not all(
            pos(n) is not None for n in ('person', 'cup', 'robot')):
        w.spin(0.3)
    have = all(pos(n) is not None for n in ('person', 'cup', 'robot'))
    step('[1] 数据链路  人=%s 杯=%s' % (xy('person'), xy('cup')), have)
    if not have:
        res['ok'] = False
        return report(node, res)

    # ---- [1b] 先让它随机走一段, 起点才不是"每次都一样"
    # 端到端要验的是"人在场地里活动, 收到指令后自己走到桌边"。若一起测就
    # 直接发指令, 人每次都从同一个初始点出发, 那条 4~5m 的路永远走同一遍,
    # 路径上的坑(比如正好撞桌腿)就被掩盖了。所以先放它自己走 WARMUP 秒。
    WARMUP = float(os.environ.get('P91_WARMUP', '25'))
    print('    让人先随机活动 %.0fs ...' % WARMUP)
    t0 = time.time()
    while time.time() - t0 < WARMUP:
        w.spin(0.3)

    p0 = xy('person')
    cup_home = P.cup_on_table()
    res['person_start'] = [round(v, 3) for v in p0]
    res['start_dist_m'] = round(math.hypot(p0[0] - L.PERSON_SPOT[0],
                                           p0[1] - L.PERSON_SPOT[1]), 2)

    # ---- [2] 下发唯一一条命令
    # 前一秒还有残留广播(DONE 心跳/上一轮的阶段), 先清零再发。
    w.reset()
    w.send('GO_DRINK')
    sim0 = w.sim
    step('[2] 已下发 GO_DRINK', True,
         '人从 (%+.2f,%+.2f) 出发, 目标 (%.2f,%.2f)'
         % (p0[0], p0[1], L.PERSON_SPOT[0], L.PERSON_SPOT[1]))

    # ---- [3] 跟踪全流程 + 关键阶段抓图
    shots_done = set()
    arrive = None
    deadline = time.time() + TOTAL_TIMEOUT
    while time.time() < deadline:
        w.spin(0.15)
        # 人真正"到位并开口"的时刻 = /robot/command 收到 DRINK(那是它说完话
        # 的产物)。person_walk 的 GOTO/DRINK 是它**自己**的状态机, 不对外播。
        if arrive is None and w.cmds:
            px, py = xy('person')
            # P95: "面向谁" 要跟**服务位**比, 不能跟机器人当前位置比 ——
            # 人开口那一刻 follower 还带着机器人在他周围绕(跟到 1.0m、还在
            # 调整), 而人刻意面向服务位: 那是 prepare_present 第一件事就要
            # place_robot 过去的地方, 也是 PERSON_SPOT / CUP_PRESENT_R 那套
            # 布局数字的前提。跟机器人现位置比, 实测会报 45° 的"没面向
            # 机器人", 其实人站得很正、递杯也很准。
            face = wrap_deg(math.degrees(
                math.atan2(L.SERVE_POSE[1] - py, L.SERVE_POSE[0] - px)
                - yaw('person')))
            arrive = {'to_spot_mm': math.hypot(px - L.PERSON_SPOT[0],
                                               py - L.PERSON_SPOT[1]) * 1000.0,
                      'face_deg': abs(face),
                      'pos': [round(px, 3), round(py, 3)]}
            print('    [人到场] pos=(%.3f, %.3f) 目标=(%.3f, %.3f) 差 %.1fmm'
                  % (px, py, L.PERSON_SPOT[0], L.PERSON_SPOT[1],
                     arrive['to_spot_mm']))
            sh = w.shot('arrive')
            if sh:
                res['shots'].append(sh)
        tag = SHOT_AT.get(w.state)
        if tag and tag not in shots_done:
            shots_done.add(tag)
            sh = w.shot(tag)
            if sh:
                res['shots'].append(sh)
        if w.state in ('DONE', 'ERROR', 'FAIL_PREPARE', 'FAIL_HANDOVER'):
            break

    res['elapsed_s'] = round(time.time() - w.t0, 1)
    # 逻辑时长: 从 GO_DRINK 到 DONE, 用 /clock 计 —— 与机器快慢无关
    res['sim_elapsed_s'] = (round(w.sim - sim0, 1)
                            if (sim0 is not None and w.sim is not None)
                            else None)
    res['stages'] = w.stages
    res['arrive'] = arrive

    c = w.cmds[-1] if w.cmds else {}
    step('[3] 语音 → 指令  收到 %d 条 DRINK  来源=%s  内容="%s"'
         % (len(w.cmds), c.get('source', '-'), c.get('text', '')),
         bool(w.cmds) and '喝水' in str(c.get('text', '')))

    if arrive:
        res['to_spot_mm'] = round(arrive['to_spot_mm'], 1)
        res['face_deg'] = round(arrive['face_deg'], 1)
        step('[4] 人走到接水位  偏差 %.1f mm  面向机器人误差 %.1f°'
             % (arrive['to_spot_mm'], arrive['face_deg']),
             arrive['to_spot_mm'] < 120.0 and arrive['face_deg'] < 20.0)
    else:
        step('[4] 人走到接水位  未进入 DRINK 阶段', False)

    need = ['PREPARE', 'PRESENT', 'HANDOVER', 'PERSON_DRINK',
            'TAKE_BACK', 'PUT_BACK', 'RETREAT', 'HANDBACK', 'DONE']
    miss = [s for s in need if s not in w.stages]
    # 用**仿真时长**判: RTF 0.3 的机器上墙上时间没有可移植性(同一份代码
    # 300s / 390s 都可能), 而"机器人自己经历了多久"才是可比的。墙上时间
    # 仍然写进日志与 JSON, 便于对照机器快慢。
    sim_s = (res['sim_elapsed_s']
             if res.get('sim_elapsed_s') is not None else res['elapsed_s'])
    step('[5] 动作全流程  阶段 %d/%d  缺 %s  逻辑用时 %.0fs(墙上 %.0fs)'
         % (len(need) - len(miss), len(need), miss or '无', sim_s,
            res['elapsed_s']),
         not miss and sim_s < 300.0)

    # ---- [6] 杯子回到原位、杯轴竖直
    Tcup = pos('cup')
    if Tcup is not None:
        d_home = float(np.linalg.norm(
            Tcup[:3, 3] - np.asarray(cup_home)[:3, 3])) * 1000.0
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, float(Tcup[2, 2])))))
        res['cup_home_mm'] = round(d_home, 1)
        res['cup_tilt_deg'] = round(tilt, 2)
        step('[6] 杯子放回原位  偏差 %.1f mm  杯轴倾 %.2f°' % (d_home, tilt),
             d_home < 10.0 and tilt < 2.0)
    else:
        step('[6] 杯子放回原位  拿不到杯子真值', False)

    # ---- [7] P93: 交权后机器人恢复跟随
    # 时间线: TAKE_BACK 之后 drink_task 就告诉人"喝完了"(人开始离开桌边),
    # 机器人自己放回杯子 → RETREAT 撤离桌边 → HANDBACK 把底盘交还 follower。
    # 所以到这里人和机器人是分开的(约 2.5m), 要验的是机器人会自己跟上去。
    #
    # 判"跟随"用**行为**而不是 follower 的某个内部状态: 机器人总路程 > 1m
    # (真的开了, 不是站着不动), 且观察末段的人机距离落回跟随带附近。
    # 人一直在随机走, 距离会波动, 所以取末段平均而不是瞬时值。
    WATCH = float(os.environ.get('P91_FOLLOW_WATCH', '75'))
    print('    交权后观察 %.0fs, 看机器人是否跟上去 ...' % WATCH)
    samples = []
    r_prev = xy('robot')
    travel = 0.0
    t0 = time.time()
    while time.time() - t0 < WATCH:
        w.spin(0.25)
        p, r = xy('person'), xy('robot')
        if p is None or r is None:
            continue
        if r_prev is not None:
            travel += math.hypot(r[0] - r_prev[0], r[1] - r_prev[1])
        r_prev = r
        samples.append((time.time() - t0,
                        math.hypot(p[0] - r[0], p[1] - r[1])))
    tail = [d for t, d in samples if t > WATCH - 15.0]
    d_end = sum(tail) / len(tail) if tail else 9e9
    in_band = [d for _t, d in samples
               if L.FOLLOW_BAND_M - L.FOLLOW_BAND_TOL_M <= d
               <= L.FOLLOW_BAND_M + L.FOLLOW_BAND_TOL_M]
    res['follow'] = {'retreat': 'RETREAT' in w.stages,
                     'handback': 'HANDBACK' in w.stages,
                     'travel_m': round(travel, 2),
                     'd_end_m': round(d_end, 2),
                     'in_band_pct': round(100.0 * len(in_band)
                                          / max(1, len(samples)), 1)}
    sh = w.shot('following')
    if sh:
        res['shots'].append(sh)
    step('[7] 交权后恢复跟随  撤离=%s 交权=%s  机器人走了 %.2fm  '
         '末段人机距离 %.2fm  在跟随带内 %.0f%%'
         % ('RETREAT' in w.stages, 'HANDBACK' in w.stages, travel, d_end,
            res['follow']['in_band_pct']),
         'HANDBACK' in w.stages and travel > 1.0 and d_end < 2.5)

    # ---- [8] P96: 人的双手真的"张开迎 → 合拢捧 → 张开交还"了吗
    # 数据来自 drink_task 广播的真值测量(两条手臂 link 的世界位姿), 不是
    # 回读关节指令 —— 人的关节不在 /drink/joint_state 里, 读不回来。
    h = w.hands
    TAN = L.CUP_GRIP_R * 1000.0          # 92mm: 手心到杯轴相切
    TOL = 25.0                           # 与 test_p96 同一套判据
    gl, gr = h.get('grip_l_mm', 9e9), h.get('grip_r_mm', 9e9)
    rl, rr = h.get('recv_l_mm', 9e9), h.get('recv_r_mm', 9e9)
    vl, vr = h.get('give_l_mm', 9e9), h.get('give_r_mm', 9e9)
    gopen, gclose = h.get('open_gap_mm', 9e9), h.get('close_gap_mm', 9e9)
    res['hands'] = dict(h)
    # recv / grip 是**同一个量**在交接前/后的两次取值。`hand_to` 是"冻结
    # 当前相对位姿"式交接, 杯子不会吸附到两手连线中点, 所以两者本该相等 ——
    # 不相等就说明交接把杯子挪了(与 [5] 的"杯心漂移"是同一件事的两种口径)。
    seam = max(abs(gl - rl), abs(gr - rr))
    ok_g = (abs(gl - TAN) < TOL and abs(gr - TAN) < TOL
            and abs(gl - gr) < 2 * TOL)
    ok_r = (abs(rl - TAN) < TOL and abs(rr - TAN) < TOL
            and abs(rl - rr) < 2 * TOL)
    ok_v = min(vl, vr) > 140.0
    ok_gap = gopen > 300.0 and 170.0 < gclose < 200.0
    step('[8] 人的双手  手心间距 张开 %.0f → 合拢 %.0fmm | 手到杯轴 交接前 '
         '%.1f/%.1f 交接后 %.1f/%.1f mm(相切 %.0f±%.0f, 前后变化 %.1fmm) '
         '| 交还时 %.0f/%.0f mm'
         % (gopen, gclose, rl, rr, gl, gr, TAN, TOL, seam, vl, vr),
         ok_g and ok_r and ok_v and ok_gap and seam < 3.0)

    step('[9] 截图 %d 张' % len(res['shots']), len(res['shots']) >= 4)

    res['ok'] = not fails
    return report(node, res)


def report(node, res):
    print('-' * 92)
    print('结论: %s' % ('通过' if res.get('ok') else
                        '不通过 (%d 项)' % len(res.get('fails', []))))
    for f in res.get('fails', []):
        print('  未通过: %s' % f)
    print('=' * 92)
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, 'test_drink.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print('结果 JSON: %s' % path)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if res.get('ok') else 2


if __name__ == '__main__':
    sys.exit(main())
