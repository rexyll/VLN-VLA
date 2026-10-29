#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""liquid.py —— 运动学液面 (P98, §6.7 第 3 项"真实液面")

之前的水 = 两个滑动副按行程插值(§2.4): 杯内水板跟着杯体一起歪(仰杯喝水时
液面是斜的)、水柱是一根从壶嘴里捅出来的伸缩杆(末端与水面之间断着 3~5cm)、
体积不守恒(倒水时长写死, 人"喝"完水还是满杯)。本模块把"水"换成一套
**运动学流体** —— 不上 SPH 求解器(4 核 VM 扛不住), 而是把液体该有的三条
可观察性质做成真:

  1. 体积守恒: 倒水量按注水速率积分, 杯内水位 = V / 内截面积;
     water_joint(水体) 仍由 GraspCarry 逐 tick 钉住(§6.4 #29), 但数值来自
     体积积分; 人喝水期间水位真实下降; 超过溢流倾角按溢流速率排水。
  2. 液面恒世界水平: 杯体倾角 > DISC_TILT 时, 液面盘(cup_water_disc)出现在
     水面高度 —— 仰杯时液面不再跟着杯子转。
  3. 水柱连续: stream0..5 六段圆柱从壶嘴伸出端逐帧铺到水面, 消除断口;
     停止倒水后统一藏到地板下(z = PARK_Z)。

为什么走 /drink/carry 的 place 指令而不是改插件:
  GraspCarry 的 `place` 按模型名**惰性解析**(ECM 每次现查, 见 GraspCarry.cc),
  所以**运行时注入**的模型不用改 C++ 插件就能逐帧驱动。6 个视觉体无碰撞、
  关重力, 全部 place 高频摆位; 隐藏 = 摆到地板下。世界文件里的同名模型由
  gen_drink_scene 的 props 块生成(同一份 builder, 见 sdf_props()), 重启世界
  后天生存在; 运行中的世界用 ensure_spawned() 注入(probe 见
  tools/liquid_probe.py, CREATE/REMOVE 实测 OK)。

水量记账: 壶体不透明、壶内水位不可见, 只记账不建模(v_kettle)。
"""
import math
import os
import subprocess
import time

import numpy as np

try:
    from vla_follow import scene_layout as L
    from vla_follow import arm_kin as K
except ImportError:                       # gen_drink_scene 直接 import 本文件
    import scene_layout as L
    import arm_kin as K

WORLD = 'follow_world'

# ---- 几何常量(与 gen_drink_scene._cup_sdf/_kettle_sdf 生成段一一对应) ----
CUP_BASE_T = 0.026                        # 杯厚底座("未倒水"的水体藏在里面)
CUP_R_IN = L.CUP_R * 0.90                 # 水体半径(与水板/液面盘同径)
A_CUP = math.pi * CUP_R_IN ** 2           # 杯内截面积 ≈ 4.07e-3 m^2
V_CUP_MAX = A_CUP * L.CUP_WATER_MAX       # 满杯 ≈ 97.7 mL
DISC_T = 0.004                            # 液面盘厚
DISC_R = CUP_R_IN
STREAM_R = 0.0055                         # 与 jet 同径
STREAM_L = 0.055                          # 单段长
STREAM_N = 6
STREAM_GAP = 0.042                        # 段间距(相邻重叠 13mm → 连续)
JET_ROD_L = 0.115                         # jet 视觉杆长(生成段同值)
SPILL_TILT = math.radians(55.0)           # 超过即按溢流排水
SPILL_RATE = 80e-6                        # m^3/s
DISC_TILT = math.radians(8.0)             # 液面盘出现阈角
POUR_FILL_S = 1.2                         # 名义注水时长(与旧行程插值一致)
POUR_WATCHDOG_S = 4.0                     # pour 卡住的兜底
STREAM_TAIL_S = 4.0                       # 忘记 pour_stop 时水柱自动收场
V_KETTLE0 = 350e-6                        # 壶内初始水量(仅记账)
LEVEL_EPS = 0.0003                        # water_joint 重发阈(米)
PARK_Z = -1.0                             # 隐藏位(地板下, 相机看不到)

DISC_NAME = 'cup_water_disc'
STREAM_NAMES = ['stream%d' % i for i in range(STREAM_N)]
LIQUID_NAMES = [DISC_NAME] + STREAM_NAMES

# 壶嘴方向(body 系): SPOUT_LOCAL → TIP_LOCAL 归一化
_sp = np.array(L.KETTLE_SPOUT_TIP_LOCAL, float) - \
    np.array(L.KETTLE_SPOUT_LOCAL, float)
SPOUT_DIR = _sp / float(np.linalg.norm(_sp))

WATER_RGBA = ('0.20 0.55 0.85', '0.20 0.55 0.85')
JET_RGBA = ('0.25 0.62 0.92', '0.25 0.62 0.92')


def level_of(v):
    """水量(m^3) → 水位(m), 截断在 [0, CUP_WATER_MAX]。"""
    return min(max(v, 0.0), V_CUP_MAX) / A_CUP


def z_align_frame(d):
    """圆柱 +z 对齐到方向 d 的 4x4(轴向对称, x/y 取任意正交基)。"""
    d = np.asarray(d, float)
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        d = np.array([0.0, 0.0, 1.0])
    else:
        d = d / n
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, d))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, d)
    x = x / float(np.linalg.norm(x))
    y = np.cross(d, x)
    return K.frame(np.column_stack([x, y, d]), (0.0, 0.0, 0.0))


# ------------------------------------------------------------------ SDF 生成
def _model_sdf(name, r, h, mat, transparency, pose):
    return ('    <model name="%s">\n'
            '      <pose>%s</pose>\n'
            '      <link name="body">\n'
            '        <gravity>false</gravity>\n'
            '        <inertial><mass>0.002</mass>\n'
            '          <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>\n'
            '            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n'
            '        <visual name="v">\n'
            '          <geometry><cylinder><radius>%s</radius>'
            '<length>%s</length></cylinder></geometry>\n'
            '          <material>\n'
            '            <ambient>%s 1</ambient>\n'
            '            <diffuse>%s 1</diffuse>\n'
            '          </material>\n'
            '          <transparency>%s</transparency>\n'
            '        </visual>\n'
            '      </link>\n'
            '    </model>\n'
            % (name, pose, r, h, mat[0], mat[1], transparency))


def sdf_props():
    """props 块用的 7 个液面模型(gen_drink_scene.gen_props 追加)。"""
    park = '0 0 %s 0 0 0' % PARK_Z
    out = ['    <!-- P98 运动学液面: 液面盘 + 水柱段(运行时由 liquid.py 逐帧\n'
           '         place 驱动; 平时停在地板下) -->\n']
    out.append(_model_sdf(DISC_NAME, DISC_R, DISC_T, WATER_RGBA, 0.35, park))
    for i in range(STREAM_N):
        out.append(_model_sdf(STREAM_NAMES[i], STREAM_R, STREAM_L,
                              JET_RGBA, 0.40, park))
    return ''.join(out)


# ------------------------------------------------------------------ 运行时注入
def _gz_req(svc, reqtype, reptype, req, tries=3):
    for _ in range(tries):
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', svc, '--reqtype', reqtype,
                 '--reptype', reptype, '--timeout', '3000', '--req', req],
                capture_output=True, text=True, timeout=15)
        except Exception:
            r = None
        if r is not None and 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.4)
    return False


def ensure_spawned(node, timeout_s=12.0):
    """把缺的液面模型注入运行中的世界(已在 links 里的跳过)。

    §2.1: 新进程对 gz 服务的发现不可靠 → 每条 create 自带 3 次重试;
    注入成功与否以 dynamic_pose 里出现实体为准(node.links 由 drink_task
    维护)。全部失败不抛错 —— 世界重启后 props 块里本来就有这批模型。
    """
    todo = [n for n in LIQUID_NAMES if n not in node.links]
    if not todo:
        return True
    t0 = time.time()
    tmpdir = '/tmp'
    for name in todo:
        sdf = _model_sdf(name,
                         DISC_R if name == DISC_NAME else STREAM_R,
                         DISC_T if name == DISC_NAME else STREAM_L,
                         WATER_RGBA if name == DISC_NAME else JET_RGBA,
                         0.35 if name == DISC_NAME else 0.40,
                         '0 0 %s 0 0 0' % PARK_Z)
        # 踩坑(实证×3): UserCommands 只读 `sdf` 字段(**内联 SDF 字符串**)。
        # 喂文件路径(无论 sdf 还是 sdf_filename)都被忽略或当 XML 解析失败,
        # 且阻塞端无应答。内联需要 protobuf-text 转义: " → \" 且换行 → \n。
        esc = sdf.replace('\\', '\\\\').replace('"', '\\"') \
            .replace('\n', '\\n')
        _gz_req('/world/%s/create/blocking' % WORLD, 'gz.msgs.EntityFactory',
                'gz.msgs.Boolean', 'sdf: "%s", name: "%s"' % (esc, name),
                tries=5)
    while time.time() - t0 < timeout_s:
        missing = [n for n in LIQUID_NAMES if n not in node.links]
        if not missing:
            node.get_logger().info('LIQUID ★ 液面模型就绪: %s'
                                   % ','.join(LIQUID_NAMES))
            return True
        time.sleep(0.2)
    node.get_logger().warn(
        'LIQUID 液面模型注入不全, 缺: %s(水位/水柱将退化为仅 water_joint)'
        % ','.join(n for n in LIQUID_NAMES if n not in node.links))
    return False


def ensure_scene(node, timeout_s=15.0):
    """道具自愈: 壶/杯不在世界里就按名义位姿重建(由 drink_task 调用)。

    为什么在 drink_task 里建而不是脚本里: 本机 gz-transport 对**新建进程**
    的服务调用不可靠(§2.1, 实测短命进程的 create/blocking 无应答、无报错),
    而 drink_task 是长驻进程 —— 液面模型就是它建成功的(实证)。重建用
    gen_drink_scene 的生成段(唯一真源, 内含名义位姿)。

    附带治好一个反复出现的故障: 壶/杯是无重力无碰撞自由体, 速度只增不减,
    早晚会漂出场外 —— 自愈逻辑会把它当"丢失"重建归位。
    """
    import importlib.util
    missing = [n for n in ('kettle', 'cup') if n not in node.links]
    if not missing:
        return True
    spec = importlib.util.spec_from_file_location(
        'gen_drink_scene',
        os.path.join(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))),
            '..', '..', 'tools', 'gen_drink_scene.py'))
    spec = importlib.util.spec_from_file_location(
        'gen_drink_scene', '/home/earl/workspace/VLA/tools/'
        'gen_drink_scene.py')
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    builders = {'kettle': g._kettle_sdf, 'cup': g._cup_sdf}
    ok = True
    for name in missing:
        # 生成段是裸 <model> 片段, ReadString 需要 <sdf> 文档包裹
        # (服务器实证: 否则 "XML does not seem to be an SDFormat")
        frag = '<sdf version="1.8">%s</sdf>' % builders[name]()
        esc = frag.replace('\\', '\\\\') \
            .replace('"', '\\"').replace('\n', '\\n')
        done = _gz_req('/world/%s/create/blocking' % WORLD,
                       'gz.msgs.EntityFactory', 'gz.msgs.Boolean',
                       'sdf: "%s", name: "%s"' % (esc, name), tries=5)
        node.get_logger().info('LIQUID ★ 重建%s: %s'
                               % (name, 'OK' if done else 'FAIL'))
        ok = done and ok
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if all(n in node.links for n in missing):
            return True
        time.sleep(0.2)
    return ok and all(n in node.links for n in missing)


# ------------------------------------------------------------------ 控制器
class LiquidCtl(object):
    """杯内水量的唯一记账处 + 液面盘/水柱的逐帧摆位。

    由 drink_task 持有; update() 挂在 50Hz 的 _carry_tick 上。
    """

    def __init__(self, node):
        self.node = node
        self.v_cup = 0.0            # 杯内水量 m^3
        self.v_kettle = V_KETTLE0   # 壶内水量(记账)
        self.mode = 'idle'          # idle | pour | drink
        self.v_rate = 0.0
        self.v_target = 0.0
        self.t_mark = time.time()
        self.drink_t0 = 0.0
        self.drink_rate = 0.0
        self.stream_show = False    # 水柱应当可见
        self.stream_until = 0.0     # 兜底收场时刻
        self.spawned = False
        self.vis = {}               # model -> 当前是否摆出
        self.last_level = None      # 上次下发的 water_joint
        self._last_send_t = 0.0     # water_joint 限流节拍
        self.last_t = time.time()

    # ---- 对外: 任务相位 ----
    def reset_cycle(self):
        """run_drink 开场: 杯内清零(water_joint 的归零仍由旧 joint_ramp 做)。"""
        self.v_cup = 0.0
        self.mode = 'idle'
        self.stream_show = False
        self._hide_all()

    def pour_start(self, fill):
        """倒水开始: 注入模型(一次性) + 启动体积积分 + 水柱亮出。"""
        if not self.spawned:
            self.spawned = ensure_spawned(self.node)
            if self.spawned:
                self.vis = {}
        self.v_target = min(A_CUP * L.CUP_WATER_MAX * float(fill), V_CUP_MAX)
        self.v_rate = self.v_target / POUR_FILL_S
        self.mode = 'pour'
        self.t_mark = time.time()
        self.stream_show = True
        self.stream_until = time.time() + 600.0   # 等 pour_stop / 看门狗
        self.node.get_logger().info(
            'LIQUID ★ 倒水开始: 目标 %.1f mL (水位 %.1f mm)'
            % (self.v_target * 1e6, level_of(self.v_target) * 1e3))

    def pour_stop(self):
        """收水完成: 水柱熄灭。"""
        self.mode = 'idle'
        self.stream_show = False

    def pour_wait(self, extra=0.0):
        """等体积注到目标(与旧 joint_ramp(dur=1.2) 等时)。"""
        t0 = time.time()
        while self.mode == 'pour' and time.time() - t0 < POUR_FILL_S + extra:
            self.node.spin_for(0.05)

    def drink_start(self, dur):
        """人开始喝: 1s 后在 dur-2s 内把水喝到剩 3%。"""
        self.mode = 'drink'
        self.drink_t0 = time.time()
        span = max(dur - 2.0, 0.5)
        self.drink_rate = max(self.v_cup - 0.03 * V_CUP_MAX, 0.0) / span

    def drink_stop(self):
        self.mode = 'idle'

    # ---- 内部 ----
    def _cup_pose(self):
        p = self.node.links.get('cup')
        if p is None:
            return None
        q = p.orientation
        return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                       (p.position.x, p.position.y, p.position.z))

    def _kettle_pose(self):
        p = self.node.links.get('kettle')
        if p is None:
            return None
        q = p.orientation
        return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                       (p.position.x, p.position.y, p.position.z))

    def _cup_tilt(self):
        T = self._cup_pose()
        if T is None:
            return 0.0
        return math.acos(max(-1.0, min(1.0, float(T[2, 2]))))

    def _surface_z(self, Tcup):
        """水面世界高度: 杯底 + 厚底座 + 水位。"""
        return (float(Tcup[2, 3]) - L.CUP_H / 2.0 + CUP_BASE_T
                + level_of(self.v_cup))

    def _send_place(self, model, T):
        self.node.carry.place_fast(model, T)
        self.vis[model] = True

    def _hide(self, model):
        if self.vis.get(model):
            T = K.frame(np.eye(3), (0.0, 0.0, PARK_Z))
            self.node.carry.place_fast(model, T)
            self.vis[model] = False

    def _hide_all(self):
        for n in LIQUID_NAMES:
            self._hide(n)

    def _drive_level(self):
        # P98 限流: /drink/carry 是所有抓取/joint 指令的**唯一通道**, 50Hz
        # 重发会把 jet 的指令淹没在 DDS 队列里(实测 jet 无 ack)。插件本来
        # 就每 tick 重发最后目标, 这里 >0.5mm 才变、且最多 ~7Hz 足够平滑。
        lvl = level_of(self.v_cup)
        now = time.time()
        if self.last_level is None:
            need = True
        else:
            need = abs(lvl - self.last_level) > 0.0005
        if need and now - self._last_send_t > 0.15:
            self.node.carry._send('joint cup water_joint %.6f' % lvl)
            self.last_level = lvl
            self._last_send_t = now

    def _update_stream(self):
        """水柱: 壶嘴伸出端 → 杯内水面, 逐帧铺段。"""
        if not (self.stream_show or self.vis.get(STREAM_NAMES[0])):
            return
        now = time.time()
        if (not self.stream_show) or \
                (self.mode == 'idle' and
                 now - self.t_mark > STREAM_TAIL_S):
            # P98: 归位必须**真值确认** —— place 是单向消息, 收水瞬间的
            # 通道拥塞会丢掉归位指令(实测 stream0/2 悬空残留), 这里按
            # 5Hz 重发直到真值落到地板下。
            if now - getattr(self, '_park_t', 0.0) > 0.2:
                dirty = False
                for n in STREAM_NAMES:
                    p = self.node.links.get(n)
                    if p is None or p.position.z > PARK_Z + 0.5:
                        T = K.frame(np.eye(3), (0.0, 0.0, PARK_Z))
                        self.node.carry.place_fast(n, T)
                        self.vis[n] = False
                        dirty = True
                self._park_t = now
                if not dirty:
                    self.vis = {k: False for k in STREAM_NAMES}
            return
        Tk = self._kettle_pose()
        Tc = self._cup_pose()
        if Tk is None or Tc is None:
            return
        tip = Tk[:3, 3] + Tk[:3, :3] @ np.array(L.KETTLE_SPOUT_TIP_LOCAL, float)
        surf = np.array([float(Tc[0, 3]), float(Tc[1, 3]),
                         self._surface_z(Tc) - 0.008])
        d = surf - tip
        lc = float(np.linalg.norm(d))
        if lc < 0.06:                      # 嘴几乎贴水面, 不需要段
            for n in STREAM_NAMES:
                self._hide(n)
            return
        u = d / lc
        n_seg = int(min(STREAM_N,
                        max(1, math.ceil((lc - STREAM_L) / STREAM_GAP) + 1)))
        for i in range(STREAM_N):
            if i < n_seg:
                c = tip + u * (STREAM_L / 2.0 + i * STREAM_GAP)
                self._send_place(STREAM_NAMES[i], K.frame(
                    z_align_frame(u)[:3, :3], (float(c[0]), float(c[1]),
                                               float(c[2]))))
            else:
                self._hide(STREAM_NAMES[i])

    def _update_disc(self, tilt):
        if DISC_NAME not in self.node.links:
            return
        if tilt < DISC_TILT:
            p = self.node.links.get(DISC_NAME)
            if p is not None and p.position.z > PARK_Z + 0.5:
                now = time.time()
                if now - getattr(self, '_disc_park_t', 0.0) > 0.2:
                    self.node.carry.place_fast(
                        DISC_NAME, K.frame(np.eye(3), (0.0, 0.0, PARK_Z)))
                    self.vis[DISC_NAME] = False
                    self._disc_park_t = now
            else:
                self.vis[DISC_NAME] = False
            return
        Tc = self._cup_pose()
        if Tc is None:
            return
        zs = self._surface_z(Tc)
        T = K.frame(np.eye(3), (float(Tc[0, 3]), float(Tc[1, 3]), zs))
        self._send_place(DISC_NAME, T)

    def _sync_park(self):
        """接管同步: 上一个进程若死在倒水中途, 会留下悬空的水柱段 ——
        新进程的 vis 表是空的, 永远不会去收, 所以这里按真值把高于地板的
        液面体全部归位一次。"""
        done = True
        for n in LIQUID_NAMES:
            p = self.node.links.get(n)
            if p is None:
                continue                     # 没注入的不用管
            if p.position.z > PARK_Z + 0.5:
                self.node.carry.place_fast(
                    n, K.frame(np.eye(3), (0.0, 0.0, PARK_Z)))
                done = False
            self.vis[n] = False
        if done:
            self._synced = True

    # ---- 50Hz 入口 ----
    def update(self):
        if not getattr(self, '_synced', False):
            self._sync_park()
        now = time.time()
        dt = min(max(now - self.last_t, 0.0), 0.1)
        self.last_t = now
        if self.mode == 'pour':
            self.v_cup = min(self.v_cup + self.v_rate * dt, V_CUP_MAX)
            self.v_kettle = max(self.v_kettle - self.v_rate * dt, 0.0)
            if self.v_cup >= self.v_target - 1e-9:
                self.mode = 'idle'
                self.t_mark = now          # fill 完成, 水柱进入残流计时
        elif self.mode == 'drink':
            if now - self.drink_t0 > 1.0:
                self.v_cup = max(self.v_cup - self.drink_rate * dt,
                                 0.03 * V_CUP_MAX)
        tilt = self._cup_tilt()
        if tilt > SPILL_TILT and self.mode != 'pour':
            self.v_cup = max(self.v_cup - SPILL_RATE * dt, 0.0)
        if self.mode == 'pour' and \
                now - self.t_mark > POUR_WATCHDOG_S:
            self.node.get_logger().warn('LIQUID 倒水看门狗触发, 强制收场')
            self.pour_stop()
        if DISC_NAME in self.node.links or self.vis:
            self._drive_level()
            self._update_stream()
            self._update_disc(tilt)
