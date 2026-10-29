"""把各检查点的关节量换算成"相对同侧肩"的口径, 用来校准 R2/R3 阈值。

人的"手肘拐进怀里"是**相对自己的肩**说的, 不是相对世界中线 —— 物体摆偏了
(实测杯子会落在基座中线左右 ~110mm), 用绝对中线判会把正常的跨体拿杯判违规。
用法: python3 tests/diag/_p103_shoulder_stats.py [json ...]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402

files = sys.argv[1:] or ['tests/out/test_p103.json']
for p in files:
    a = (json.load(open(p, encoding='utf-8'))['result'].get('arm') or {})
    print('==', p)
    print('%-10s %s  %8s %8s %8s' % ('tag', 'side', 'dE', 'dP', '外扩'))
    for tag in sorted(a):
        for side, st in sorted(a[tag].items()):
            sgn = 1.0 if side == 'L' else -1.0
            sy = K.SHOULDER[side][1]
            q = np.asarray(st['q'], float)
            el = K.elbow_pos(side, q)[1] * 1e3
            pa = K.fk(side, q)[1, 3] * 1e3
            print('%-10s %s  %+8.0f %+8.0f %8.0f  %s'
                  % (tag, side, sgn * (el - sy * 1e3), sgn * (pa - sy * 1e3),
                     st['out_line_mm'], ','.join(st['bad']) or '-'))
