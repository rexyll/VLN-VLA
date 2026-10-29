#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p98.py —— P98 运动学液面验收(§6.7 第 3 项: 真实液面)

对"水"的三条可观察性质做端到端判定(完整喝水栈上触发一轮 GO_DRINK):

  [1] 模型就绪: 7 个液面模型(液面盘 + 6 段水柱)全部存在于 dynamic_pose。
  [2] 倒水水柱: 倒水窗口(PREPARE 后段)内 ≥2 段 stream 悬在桌面上方
      (z > 0.60, 即壶嘴→水面之间真的铺出了连续水柱)。
  [3] 喝水消耗: PERSON_DRINK 期间液面盘可见(杯被仰起 >8°)且**水位真实
      下降** —— 盘高-杯底换算出的水位在喝前/喝后差 ≥3mm(旧滑动副方案
      里"人喝完水还是满杯", 这是本项验收的核心)。
  [4] 收尾归位: DONE 之后 7 个模型全部停到地板下(z < -0.5), 不留悬空残骸。

前提: start_scene.sh + start_drink_demo.sh(NAV2=1) + person_walk
PW_AUTO_DRINK=0(本测试自己触发)。产物: vla_ws/logs/test_p98.json
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
from std_msgs.msg import String

import gz.transport13 as gz_transport
import gz.msgs10.pose_v_pb2 as pose_v

VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
WORLD = 'follow_world'
OUT = os.path.join(VLA_HOME, 'vla_ws', 'logs', 'test_p98.json')
LIQUID = ['cup_water_disc'] + ['stream%d' % i for i in range(6)]
CUP_H = 0.100
CUP_BASE_T = 0.026


class Mon(Node):
    def __init__(self):
        super().__init__('p98_mon')
        self.state = '?'
        self.create_subscription(String, '/drink/state', self._st, 10)
        self.pub_person = self.create_publisher(String, '/person/command', 10)

    def _st(self, m):
        try:
            self.state = json.loads(m.data).get('state', m.data)
        except Exception:
            self.state = m.data

    def go_drink(self):
        for _ in range(2):
            self.pub_person.publish(String(data=json.dumps(
                {'action': 'GO_DRINK'})))
            time.sleep(0.5)


class Truth(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.p = {}
        self.gz = gz_transport.Node()
        self.gz.subscribe(pose_v.Pose_V,
                          '/world/%s/dynamic_pose/info' % WORLD, self._cb)
        self.start()

    def _cb(self, msg):
        d = {}
        for p in msg.pose:
            d[p.name] = (p.position.x, p.position.y, p.position.z)
        with self.lock:
            self.p.update(d)

    def get(self, name):
        with self.lock:
            return self.p.get(name)


def flog_len(path, tag_skip=0):
    try:
        with open(path, encoding='utf-8', errors='ignore') as f:
            return sum(1 for _ in f)
    except Exception:
        return 0


def flog_has(path, tag, since):
    try:
        with open(path, encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        return any(tag in l for l in lines[since:])
    except Exception:
        return False


def main():
    rclpy.init()
    mon = Mon()
    threading.Thread(target=rclpy.spin, args=(mon,), daemon=True).start()
    tr = Truth()
    res = {}
    drink_log = os.path.join(VLA_HOME, 'vla_ws', 'logs', 'drink.log')
    pw_log = os.path.join(VLA_HOME, 'vla_ws', 'logs', 'pw.log')

    def level_mm():
        """水位(mm) = 液面盘高 - 杯底 - 厚底座。盘只在杯倾 >8° 时可见,
        而杯倾只发生在人喝水的窗口 —— 天然就是"喝水消耗"的观测窗。"""
        d = tr.get('cup_water_disc')
        c = tr.get('cup')
        if not d or not c:
            return None
        return (d[2] - (c[2] - CUP_H / 2.0) - CUP_BASE_T) * 1000.0

    # 布景: 等真值
    t0 = time.time()
    while time.time() - t0 < 12:
        if tr.get('robot') and tr.get('cup'):
            break
        time.sleep(0.2)

    # 触发(带 pw 日志确认 —— 新进程的 GO_DRINK 前几条会丢在发现期 #24)
    nl_pw = flog_len(pw_log)
    t0 = time.time()
    sent = 0
    while time.time() - t0 < 12.0:
        mon.go_drink()
        sent += 1
        if flog_has(pw_log, '收到"去喝水"', nl_pw):
            break
        time.sleep(0.6)
    print('[P98] GO_DRINK: %s (%d 条)' % ('已确认' if flog_has(
        pw_log, '收到"去喝水"', nl_pw) else '未确认!', sent), flush=True)

    # [1] 模型就绪
    ok1 = all(tr.get(n) for n in LIQUID)
    t0 = time.time()
    while time.time() - t0 < 20 and not ok1:
        ok1 = all(tr.get(n) for n in LIQUID)
        time.sleep(0.2)
    res['1'] = {'ok': bool(ok1),
                'missing': [n for n in LIQUID if not tr.get(n)]}
    print('[P98][1] 模型就绪 → %s' % ('PASS' if ok1 else 'FAIL'), flush=True)

    # [2]+[3] 全程连续采样(不依赖任何 ROS 状态订阅):
    #   水柱可见峰值 ≥2 段 = 倒水真实铺出;
    #   液面盘只在杯倾 >8°(人喝水)时可见, 盘水位从高到低降 ≥3mm = 喝水真实消耗。
    nl_dk = flog_len(drink_log)
    t0 = time.time()
    max_streams = 0
    lv_max = None
    lv_min_after = None
    done = False
    while time.time() - t0 < 300.0:
        vis = sum(1 for n in STREAM_VIS
                  if (tr.get(n) or (9, 9, -9))[2] > 0.60)
        max_streams = max(max_streams, vis)
        lv = level_mm()
        if lv is not None:
            if lv_max is None or lv > lv_max:
                lv_max = lv
            elif lv_min_after is None or lv < lv_min_after:
                lv_min_after = lv
        if flog_has(drink_log, 'DRINK ★ 完成', nl_dk):
            done = True
            break
        time.sleep(0.25)
    res['2'] = {'ok': bool(max_streams >= 2), 'max_stream_vis': max_streams}
    print('[P98][2] 倒水水柱峰值 %d 段 → %s'
          % (max_streams, 'PASS' if max_streams >= 2 else 'FAIL'), flush=True)
    drop = (lv_max - lv_min_after) if (lv_max is not None
                                       and lv_min_after is not None) else None
    res['3'] = {'ok': bool(drop is not None and drop >= 3.0),
                'level_max_mm': round(lv_max, 1) if lv_max is not None else None,
                'level_min_mm': round(lv_min_after, 1)
                if lv_min_after is not None else None,
                'drop_mm': round(drop, 1) if drop is not None else None}
    print('[P98][3] 喝水消耗: 水位 %.1f→%.1fmm (降 %s) → %s'
          % (lv_max or -1, lv_min_after or -1, res['3']['drop_mm'],
             'PASS' if res['3']['ok'] else 'FAIL'), flush=True)

    # [4] 收尾归位(DONE 后 7 个模型全部停到地板下)
    if not done:
        done = wait_done = flog_has(drink_log, 'DRINK ★ 完成', nl_dk)
        t4 = time.time()
        while time.time() - t4 < 60 and not done:
            done = flog_has(drink_log, 'DRINK ★ 完成', nl_dk)
            time.sleep(0.5)
    time.sleep(3.0)
    high = [n for n in LIQUID if (tr.get(n) or (9, 9, 9))[2] > -0.5]
    res['4'] = {'ok': bool(done and not high), 'leftover': high}
    print('[P98][4] 残留悬空 %s → %s'
          % (high or '无', 'PASS' if not high else 'FAIL'), flush=True)

    all_ok = all(v.get('ok') for v in res.values() if isinstance(v, dict))
    res['_ok'] = bool(all_ok)
    try:
        with open(OUT, 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
    except Exception as e:
        print('[P98] 写产物失败: %r' % e, flush=True)
    print('[P98] ===== %s =====' % ('ALL PASS' if all_ok else 'FAILED'),
          flush=True)
    os._exit(0 if all_ok else 1)


STREAM_VIS = ['stream%d' % i for i in range(6)]

if __name__ == '__main__':
    main()
