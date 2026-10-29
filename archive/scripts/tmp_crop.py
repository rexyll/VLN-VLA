#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Gazebo 桌面截图裁到 3D 视图区域并放大, 便于查看轨迹。"""
import sys

import gi

gi.require_version('GdkPixbuf', '2.0')
from gi.repository import GdkPixbuf  # noqa: E402

src_path, dst_path = sys.argv[1], sys.argv[2]
x, y, w, h = (int(v) for v in sys.argv[3:7])
scale = float(sys.argv[7]) if len(sys.argv) > 7 else 1.6

src = GdkPixbuf.Pixbuf.new_from_file(src_path)
w = min(w, src.get_width() - x)
h = min(h, src.get_height() - y)
sub = GdkPixbuf.Pixbuf.new_subpixbuf(src, x, y, w, h)
out = sub.scale_simple(int(w * scale), int(h * scale), GdkPixbuf.InterpType.BILINEAR)
out.savev(dst_path, 'png', [], [])
print('CROP_OK %s %dx%d' % (dst_path, out.get_width(), out.get_height()))
