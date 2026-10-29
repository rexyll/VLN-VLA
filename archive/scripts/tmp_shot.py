#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取本机桌面（Gazebo 窗口就在桌面上），保存为 PNG。
用法: DISPLAY=:0 python3 tmp_shot.py /path/out.png
"""
import sys

import gi

gi.require_version('Gdk', '3.0')
from gi.repository import Gdk  # noqa: E402

path = sys.argv[1]
win = Gdk.get_default_root_window()
w, h = win.get_width(), win.get_height()
pb = Gdk.pixbuf_get_from_window(win, 0, 0, w, h)
if pb is None:
    print('SHOT_FAIL')
    sys.exit(1)
pb.savev(path, 'png', [], [])
print('SHOT_OK %s %dx%d' % (path, w, h))
