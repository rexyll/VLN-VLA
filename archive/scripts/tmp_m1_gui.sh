#!/bin/bash
# tmp_m1_gui.sh —— gui.config 插入 ImageDisplay 面板(MinimalScene 之后, 保证锚定目标存在)
python3 - <<'PYEOF'
import re
p = '/home/earl/.gz/sim/8/gui.config'
s = open(p, encoding='utf-8').read()
if 'ImageDisplay' in s:
    # 已存在: 先移除旧块再重插(幂等)
    s = re.sub(r'<plugin filename="ImageDisplay".*?</plugin>\n?', '', s, flags=re.S)
block = '''
<!-- 前置相机画面小窗: 锚定主视图左上角, 不遮挡全局俯视 -->
<plugin filename="ImageDisplay" name="Camera View">
    <gz-gui>
        <property type="bool" key="showTitleBar">true</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="width">330</property>
        <property type="double" key="height">260</property>
        <property type="double" key="z">2</property>
        <property type="string" key="state">floating</property>
        <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="top" target="top"/>
        </anchors>
    </gz-gui>
    <topic>/camera</topic>
    <height>240</height>
    <width>320</width>
</plugin>
'''
# 插到 MinimalScene 块结束后(第一个 </plugin> 且其块内含 MinimalScene)
m = re.search(r'<plugin filename="MinimalScene".*?</plugin>\n?', s, flags=re.S)
assert m, 'MinimalScene not found'
s = s[:m.end()] + block + s[m.end():]
open(p, 'w', encoding='utf-8').write(s)
print('gui.config updated, ImageDisplay inserted after MinimalScene')
PYEOF
grep -n 'camera_pose\|ImageDisplay' /home/earl/.gz/sim/8/gui.config | head -4
