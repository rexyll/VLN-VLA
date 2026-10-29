#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 depth_percept 从"4 个方向各一个数"改成"8 个 45° 方位各一个数"。

为什么要改:
  gpu_lidar 在本项目里是死的 —— 为了绕开 VMware 下 ogre2 白屏, 启动脚本固定
  用 --render-engine ogre(ogre1), 而 GpuLidar 依赖 ogre2 的 GPU 深度预渲染。
  实测 /lidar 的 360 条射线 min=中位=max=0.120(正好是 range_min), 一条 inf 都
  没有; 换 type="lidar"(CPU) 又被 gz-sim8 拒绝: "Sensor type LIDAR not
  supported yet"。所以雷达在这个环境里不可用, 只能退回 4 台深度相机。

  但原来的 depth_percept 把每台相机中心 80% 的列**揉成一块**取 10% 分位数,
  一个只占画面边缘几列的斜前方立柱会被大片背景稀释掉, front 报得很远 ——
  这就是"斜着走过去有时能绕、有时直接撞"的直接原因。

改动:
  1. 每台相机按列分 SECT_N=5 个扇区(每扇区 20°), 各自取 5% 分位数。
  2. 按相机安装方位把 20 个扇区合成到 8 个 45° 方位输出:
     front / fl / left / bl / back / br / right / fr
"""
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/depth_percept.py'
shutil.copy(P, P + '.bak_sect')
s = open(P, encoding='utf-8').read()


def rep(old, new, tag):
    global s
    if s.count(old) != 1:
        print('!! 匹配失败(%s): 出现 %d 次' % (tag, s.count(old)))
        sys.exit(1)
    s = s.replace(old, new)


# ---- 1. 常量 ----
rep("MAX_D = 12.0",
    "MAX_D = 12.0\n"
    "SECT_N = 5          # 每台相机水平分 5 个扇区(每个 20°)\n"
    "SECT_Q = 0.05       # 扇区内取 5% 分位数(扇区窄, 不怕被背景稀释)",
    '常量')

# ---- 2. 状态 ----
rep("        self.depth = {n: -1.0 for n, _ in DIRS}\n"
    "        self.stamp = {n: 0.0 for n, _ in DIRS}",
    "        self.depth = {n: -1.0 for n, _ in DIRS}\n"
    "        self.stamp = {n: 0.0 for n, _ in DIRS}\n"
    "        # P26: 每台相机 5 个扇区的最近距离(左→右, 中心方位 c+40,c+20,c,c-20,c-40)\n"
    "        self.sect = {n: [-1.0] * SECT_N for n, _ in DIRS}",
    '状态')

# ---- 3. 深度回调: 分扇区 ----
rep("""            a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4')
            # 上带视场(行10%~45%): 该带射线均在水平线以上,
            # 排除地面与机器人自身底盘的自遮挡; 列取中心 80%
            r0, r1 = int(h * 0.10), int(h * 0.45)
            c0, c1 = int(w * 0.10), int(w * 0.90)
            a = a.reshape(h, w)[r0:r1, c0:c1].ravel()
            valid = a[np.isfinite(a) & (a > 0.05) & (a < 11.5)]
            # RGBA 帧按 float32 解析会得到极小值, 有效点不足 → 判为非深度帧
            if valid.size < 30:
                return
            d = float(np.quantile(valid, 0.10))
            with self._lock:
                self.depth[name] = d
                self.stamp[name] = time.monotonic()""",
    """            a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4')
            # 上带视场(行10%~45%): 该带射线均在水平线以上,
            # 排除地面与机器人自身底盘的自遮挡
            r0, r1 = int(h * 0.10), int(h * 0.45)
            a = a.reshape(h, w)[r0:r1, :]
            ok = np.isfinite(a) & (a > 0.05) & (a < 11.5)
            # RGBA 帧按 float32 解析会得到极小值, 有效点不足 → 判为非深度帧
            if int(ok.sum()) < 60:
                return
            # P26: 逐扇区取分位数, 保留方位信息(原实现把中心 80% 列揉成
            # 一块取 10% 分位数, 只占画面边缘的斜前方立柱会被背景稀释掉)
            sec = []
            for k in range(SECT_N):
                c0, c1 = int(w * k / SECT_N), int(w * (k + 1) / SECT_N)
                v = a[:, c0:c1][ok[:, c0:c1]]
                sec.append(float(np.quantile(v, SECT_Q)) if v.size >= 15
                           else -1.0)
            with self._lock:
                self.sect[name] = sec
                self.depth[name] = min([x for x in sec if x > 0] or [-1.0])
                self.stamp[name] = time.monotonic()""",
    '深度回调')

# ---- 4. 发布: 8 方位 ----
rep("""    def _tick(self):
        now = time.monotonic()
        out, ages = {}, {}
        with self._lock:
            for n, _ in DIRS:
                age = now - self.stamp[n]
                ages[n] = age
                out[n] = self.depth[n] if age < 0.6 else -1.0
        # P25 修正: 原为 max(ages.values()) —— 只要 4 台相机里有任意一台不推流,
        # age 就永远 >0.6, follower 的"层1 深度避障"整层被门控掉, 且完全不报错。
        # 各方向的新鲜度已由上面的 out[n]=-1 单独表达, 这里只回报 front 的帧龄。
        out['age'] = round(ages.get('front', 99.0), 3)
        msg = String()
        msg.data = json.dumps(out)
        self.pub.publish(msg)""",
    """    def _tick(self):
        now = time.monotonic()
        with self._lock:
            ages = {n: now - self.stamp[n] for n, _ in DIRS}
            sec = {n: list(v) for n, v in self.sect.items()}

        def g(name, idx):
            if ages[name] >= 0.6:
                return -1.0
            v = sec.get(name) or []
            return v[idx] if len(v) > idx and v[idx] > 0 else -1.0

        def mn(*vals):
            vs = [v for v in vals if v > 0]
            return min(vs) if vs else -1.0

        # 合成 8 个 45° 方位。各相机扇区中心方位 = 安装方位 + (+40,+20,0,-20,-40)
        #   front(0°)  : [+40, +20,   0, -20, -40]
        #   left (+90°): [+130,+110, +90, +70, +50]
        #   right(-90°): [-50, -70, -90,-110,-130]
        #   back (180°): [-140,-160, 180,+160,+140]
        out = {
            'front': mn(g('front', 1), g('front', 2), g('front', 3)),
            'fl':    mn(g('front', 0), g('left', 4)),
            'left':  mn(g('left', 1), g('left', 2), g('left', 3)),
            'bl':    mn(g('left', 0), g('back', 4)),
            'back':  mn(g('back', 1), g('back', 2), g('back', 3)),
            'br':    mn(g('back', 0), g('right', 4)),
            'right': mn(g('right', 1), g('right', 2), g('right', 3)),
            'fr':    mn(g('right', 0), g('front', 4)),
        }
        # P25: 帧龄只回报 front(各向新鲜度已由 -1 单独表达)
        out['age'] = round(ages.get('front', 99.0), 3)
        msg = String()
        msg.data = json.dumps(out)
        self.pub.publish(msg)""",
    '发布')

open(P, 'w', encoding='utf-8').write(s)
print('OK: depth_percept → 8 方位输出 (SECT_N=%d, 分位数 %.2f)' % (5, 0.05))
