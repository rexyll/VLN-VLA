#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reset_props.py —— 把水壶/水杯摆回场景布局的名义位(调试/复位用)。

用法: python3 tools/reset_props.py
坐标取 scene_layout 的唯一真源(KETTLE_*/CUP_*), z 用各自的模型原点高度。
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'vla_ws', 'src', 'vla_follow'))
from vla_follow import scene_layout as L          # noqa: E402
from vla_follow import carry as C                 # noqa: E402  (quat_of)

import numpy as np                                 # noqa: E402
import subprocess                                  # noqa: E402
import time                                        # noqa: E402

WORLD = 'follow_world'


def gz_set_pose(name, x, y, z, yaw):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(5):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            print('OK %s -> (%.3f, %.3f, z=%.3f, yaw=%.3f)'
                  % (name, x, y, z, yaw))
            return True
        time.sleep(0.4)
    print('FAIL set_pose %s' % name)
    return False


def _truth_of(names, window=3.0):
    """短暂订阅 dynamic_pose, 返回 {name: (x,y,z)}(发现不可靠时返回 {})。"""
    import gz.transport13 as gz_transport
    import gz.msgs10.pose_v_pb2 as pose_v
    got = {}
    node = gz_transport.Node()

    def cb(msg):
        for p in msg.pose:
            if p.name in names and p.name not in got:
                got[p.name] = (p.position.x, p.position.y, p.position.z)
    node.subscribe(pose_v.Pose_V,
                   '/world/%s/dynamic_pose/info' % WORLD, cb)
    t0 = time.time()
    while time.time() - t0 < window and len(got) < len(names):
        time.sleep(0.1)
    return got


def release_carry(timeout=12.0):
    """松掉插件里残留的抓握目标 + 归零两个水关节(等到 ack 为止)。

    崩溃的 drink_task 不会撤销已登记的 `set` 目标 —— GraspCarry 会每 1ms
    把壶/杯钉回最后目标, 任何摆位复位都必须先确认 release 生效。
    教训(实测): 新建发布者最初几条消息会丢在 DDS 发现期(#24), 盲发三连发
    等于没发 —— 这里订阅 /drink/carry/ack, 重发直到看到 ok。
    """
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('reset_props_pub')
    pub = n.create_publisher(String, '/drink/carry', 10)
    got = {}

    def _ack(m):
        got['ack'] = m.data
    n.create_subscription(String, '/drink/carry/ack', _ack, 10)
    want = [('release all', 'ok release all'),
            ('joint cup water_joint 0.000000', 'ok joint water_joint'),
            ('joint kettle jet_joint 0.000000', 'ok joint jet_joint')]
    for txt, ok_prefix in want:
        t0 = time.time()
        while time.time() - t0 < timeout:
            pub.publish(String(data=txt))
            t1 = time.time()
            while time.time() - t1 < 0.4:
                rclpy.spin_once(n, timeout_sec=0.05)
            if got.get('ack', '').startswith(ok_prefix):
                print('ACK %s' % ok_prefix)
                break
        else:
            print('WARN 未收到 ack: %s (插件侧没生效?)' % ok_prefix)
    n.destroy_node()
    rclpy.shutdown()
    print('carry released')


def _gz_req(svc, reqtype, reptype, req, tries=3):
    for _ in range(tries):
        r = subprocess.run(
            ['gz', 'service', '-s', svc, '--reqtype', reqtype,
             '--reptype', reptype, '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.4)
    return False


def rebind_joints():
    """重建道具后把插件的 joints 登记换绑到新实体。

    踩坑(实测): 插件的 joints map 没有任何运行时清除指令 —— 道具被删后,
    旧的 water_joint/jet_joint 登记还指向死实体, PreUpdate 每 tick 报
    "Trying to create a component ... entity does not exist"(日志每毫秒
    两行, 30 分钟刷了 106MB)。重新 `joint` 一次会用**新解析的实体**覆盖
    旧登记, 洪水即止。
    """
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('reset_props_bind')
    pub = n.create_publisher(String, '/drink/carry', 10)
    got = {}

    def _ack(m):
        got['ack'] = m.data
    n.create_subscription(String, '/drink/carry/ack', _ack, 10)
    for txt, okp in (('joint cup water_joint 0.000000',
                      'ok joint water_joint'),
                     ('joint kettle jet_joint 0.000000',
                      'ok joint jet_joint')):
        t0 = time.time()
        while time.time() - t0 < 10.0:
            pub.publish(String(data=txt))
            t1 = time.time()
            while time.time() - t1 < 0.4:
                rclpy.spin_once(n, timeout_sec=0.05)
            if got.get('ack', '').startswith(okp):
                print('REBIND %s' % okp)
                break
        else:
            print('WARN rebind 无 ack: %s' % okp)
    n.destroy_node()
    rclpy.shutdown()


def recreate_props():
    """删除并重建壶/杯。

    为什么不是摆位: 壶/杯是**无重力、无碰撞**的自由体(§2.2 约定), 一旦从
    任何来源(JointPositionReset 的约束脉冲 / teleport 残速)获得速度就
    **永远不衰减** —— 实测复位后 0.5s 内又漂走 116mm。删了重造的模型
    速度为零、关节状态为零, 才是真正的"复位"。
    SDF 直接取 gen_drink_scene 的生成段(与场景文件同一真源)。

    踩坑(服务器日志实证): EntityFactory 的 `sdf` 字段是 **SDF 字符串**,
    `sdf_filename` 才是文件路径 —— 喂路径会报 "Unable to read SDF string:
    /tmp/xxx.sdf"(把路径当 XML 解析)。
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'gen_drink_scene',
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     'gen_drink_scene.py'))
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    for name in ('cup', 'kettle'):
        _gz_req('/world/%s/remove' % WORLD, 'gz.msgs.Entity',
                'gz.msgs.Boolean', 'name: "%s", type: MODEL' % name)
    time.sleep(0.5)
    ok = True
    for name, sdf in (('kettle', g._kettle_sdf()),
                      ('cup', g._cup_sdf())):
        # 踩坑(实证): UserCommands 只读 `sdf` 字段(**内联 SDF 字符串**),
        # `sdf_filename` 会被静默忽略 —— 请求到达但没有模型、没有报错。
        # 内联需要 protobuf-text 转义: " → \" 且换行 → \n。
        frag = '<sdf version="1.8">%s</sdf>' % sdf
        esc = frag.replace('\\', '\\\\').replace('"', '\\"') \
            .replace('\n', '\\n')
        ok_c = _gz_req('/world/%s/create/blocking' % WORLD,
                       'gz.msgs.EntityFactory', 'gz.msgs.Boolean',
                       'sdf: "%s", name: "%s"' % (esc, name), tries=5)
        print('CREATE %s: %s' % (name, 'OK' if ok_c else 'FAIL'))
        ok = ok_c and ok
    if ok:
        time.sleep(0.5)
        rebind_joints()
    return ok


def main():
    release_carry()
    time.sleep(0.3)
    recreate_props()
    time.sleep(0.8)
    got = _truth_of(['kettle', 'cup'])
    for name, (nx, ny, nz) in (('kettle', (L.KETTLE_XY[0], L.KETTLE_XY[1],
                                           L.KETTLE_CZ)),
                               ('cup', (L.CUP_XY[0], L.CUP_XY[1],
                                        L.CUP_CZ))):
        if name not in got:
            print('VERIFY %s: 真值订阅未取到(发现期), 请人工复核' % name)
            continue
        x, y, z = got[name]
        d = math.hypot(x - nx, y - ny) + abs(z - nz)
        print('VERIFY %s: 真值 (%.3f, %.3f, %.3f) 偏差 %.1f mm %s'
              % (name, x, y, z, d * 1e3, 'OK' if d < 0.02 else '★NG'))
    sys.exit(0)


if __name__ == '__main__':
    main()
