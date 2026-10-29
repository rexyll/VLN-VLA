#!/bin/bash
set +u
cd /home/earl/workspace/VLA || exit 1
python3 vla_demo/gz_gui_fix.py
echo "Overhead 出现次数: $(grep -c 'Overhead View' ~/.gz/sim/8/gui.config)"
python3 - <<'PY'
import xml.dom.minidom as m
m.parse('/home/earl/.gz/sim/8/gui.config')
print('cfg xml OK')
PY
