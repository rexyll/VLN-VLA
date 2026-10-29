#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_draw_fov.py —— 按实测内参画 4 相机视野覆盖图

数值全部来自运行中 Gazebo 的 /depth_*/camera_info 反推(见 check_camera_fov.py):
    fx = fy = 134.26 px,  320x240
    hfov = 2*atan(320/(2*134.26)) = 100.000 deg
    安装间隔 90 deg  ->  相邻重叠 100-90 = 10.000 deg
    覆盖 7200/7200 采样点 = 100%
"""
import math

from PIL import Image, ImageDraw, ImageFont

W = H = 1180
CX, CY = W // 2, H // 2 + 20
R = 400                      # 半径像素 = 4 m
HFOV = 100.0
N = 4
OFF = 90.0
OVER = HFOV - OFF            # 10.0

BG = (250, 251, 253)
INK = (32, 38, 48)
INK2 = (108, 118, 132)
GRID = (226, 231, 238)

CAMS = [('front', 0.0, (26, 158, 82), (16, 116, 60), '前 FRONT'),
        ('left', 90.0, (40, 118, 226), (22, 86, 176), '左 LEFT'),
        ('back', 180.0, (146, 84, 216), (112, 58, 176), '后 BACK'),
        ('right', 270.0, (222, 168, 20), (176, 128, 8), '右 RIGHT')]

img = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(img)

# ---- 字体 ----
def font(sz):
    for p in ('C:/Windows/Fonts/msyhbd.ttc', 'C:/Windows/Fonts/msyh.ttc',
              'C:/Windows/Fonts/simhei.ttf'):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            continue
    return ImageFont.load_default()

f_title = font(30)
f_sub = font(17)
f_lab = font(21)
f_sm = font(15)
f_deg = font(16)


def pol(ang_deg, r):
    """数学角(0=前, 逆时针为正) -> 屏幕坐标; 屏幕 y 向下, 故取负"""
    a = math.radians(ang_deg)
    return CX + r * math.cos(a), CY - r * math.sin(a)


# ---- 极坐标网格 ----
for rr in (R * 0.25, R * 0.5, R * 0.75, R):
    d.ellipse([CX - rr, CY - rr, CX + rr, CY + rr], outline=GRID, width=1)
for a in range(0, 360, 30):
    d.line([pol(a, R * 0.06), pol(a, R)], fill=GRID, width=1)

# ---- 视野扇形（叠加混合） ----
lay = Image.new('RGBA', (W, H), (0, 0, 0, 0))
dl = ImageDraw.Draw(lay)
for _n, base, col, _c2, _txt in CAMS:
    a0 = base - HFOV / 2.0
    pts = [pol(a0 + HFOV * i / 90.0, R) for i in range(91)]
    dl.polygon([(CX, CY)] + pts, fill=col + (78,))

# 相邻重叠区（10°）用更深的色再叠一层
for _n, base, _c, c2, _t in CAMS:
    a_mid = base + OFF / 2.0          # 相邻两台的交界
    pts = [pol(a_mid - OVER / 2.0 + OVER * i / 40.0, R) for i in range(41)]
    dl.polygon([(CX, CY)] + pts, fill=c2 + (60,))
img = Image.alpha_composite(img.convert('RGBA'), lay).convert('RGB')
d = ImageDraw.Draw(img)

# ---- 扇形边界线 ----
for _n, base, _c, c2, _t in CAMS:
    for s in (-1, 1):
        d.line([(CX, CY), pol(base + s * HFOV / 2.0, R)], fill=c2, width=2)

# ---- 机器人本体 ----
d.ellipse([CX - 18, CY - 12, CX + 18, CY + 12], fill=(40, 48, 60),
          outline=(255, 255, 255), width=2)
d.polygon([(CX + 18, CY), (CX + 34, CY - 8), (CX + 34, CY + 8)],
          fill=(40, 48, 60))       # 车头指向 0°

# ---- 相机标签 ----
for _n, base, col, c2, txt in CAMS:
    x, y = pol(base, R * 0.72)
    tw = d.textlength(txt, font=f_lab)
    d.rounded_rectangle([x - tw / 2 - 12, y - 17, x + tw / 2 + 12, y + 17],
                        radius=8, fill=(255, 255, 255), outline=c2, width=2)
    d.text((x - tw / 2, y - 13), txt, font=f_lab, fill=c2)

# ---- 重叠角标注（每处 10.00°） ----
for _n, base, _c, c2, _t in CAMS:
    a_mid = base + OFF / 2.0
    x1, y1 = pol(a_mid + OVER / 2.0, R * 1.04)
    x2, y2 = pol(a_mid - OVER / 2.0, R * 1.04)
    d.line([(x1, y1), (x2, y2)], fill=c2, width=3)
    tx, ty = pol(a_mid, R * 1.14)
    s = '重叠 10.00°'
    tw = d.textlength(s, font=f_sm)
    d.rounded_rectangle([tx - tw / 2 - 9, ty - 13, tx + tw / 2 + 9, ty + 13],
                        radius=7, fill=(255, 255, 255), outline=INK2, width=1)
    d.text((tx - tw / 2, ty - 9), s, font=f_sm, fill=INK)

# ---- 半径刻度 ----
for frac in (1.0, 2.0, 3.0, 4.0):
    x, y = pol(300.0, R * frac / 4.0)
    d.text((x + 5, y - 8), f'{frac:.0f} m', font=f_sm, fill=INK2)

# ---- 标题与结论 ----
d.text((42, 30), '4 路 RGB-D 相机视野覆盖', font=f_title, fill=INK)
d.text((42, 72), '依据运行中 Gazebo 的 /depth_*/camera_info 内参反推，非读 SDF 数值',
       font=f_sub, fill=INK2)


def bullet(y, k, v, strong=False):
    d.ellipse([44, y + 8, 52, y + 16], fill=(26, 158, 82))
    d.text((62, y), k, font=f_sm, fill=INK2)
    d.text((62 + d.textlength(k, font=f_sm) + 8, y - 3), v,
           font=(f_lab if strong else f_sm), fill=INK)


bullet(112, '单台水平视场 ', '100.000°', True)
# 第二行拆成三段画, 避免中文与数字宽度不同导致重叠
_x = 62
for _s, _f, _c in (('安装间隔 ', f_sm, INK2),
                   ('90.000°', f_sm, INK),
                   ('   →   相邻重叠 ', f_sm, INK2)):
    d.text((_x, 142), _s, font=_f, fill=_c)
    _x += d.textlength(_s, font=_f)
d.text((_x, 139), '10.000°', font=f_lab, fill=(26, 158, 82))
bullet(172, '360° 覆盖 ', '100 %  (7200 / 7200 采样点)', True)
bullet(202, '内参 ', 'fx = fy = 134.26 px   320×240  update 10 Hz', False)
bullet(232, '状态 ', '满足要求：完整覆盖 + 相邻重叠约 10°', True)

# ---- 图例 ----
lx, ly = 42, H - 118
d.text((lx, ly - 26), '图例', font=f_sm, fill=INK2)
for i, (_n, _b, col, c2, txt) in enumerate(CAMS):
    x = lx + i * 190
    d.rounded_rectangle([x, ly, x + 26, ly + 16], radius=4,
                        fill=col, outline=c2)
    d.text((x + 34, ly - 2), txt, font=f_sm, fill=INK)
d.rounded_rectangle([lx, ly + 34, lx + 26, ly + 50], radius=4,
                    fill=(46, 122, 196), outline=(22, 86, 176))
d.text((lx + 34, ly + 32), '深色叠加带 = 两路重叠区（10°）', font=f_sm, fill=INK)
d.text((lx + 380, ly + 32),
       '扇形半径 4 m；车头指向正前（0°）', font=f_sm, fill=INK)

img.save(r'F:\workspace\_coding_workspace\VLA\fov_coverage.png')
print('OK fov_coverage.png', img.size, 'overlap=%.3f°' % OVER)
