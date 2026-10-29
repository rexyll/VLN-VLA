"""打印某次验收 JSON 里各检查点的实测关节位形(一行一站)。

用法: python3 tests/diag/_p103_show_arm.py [tests/out/test_p103.json] [tag ...]
"""
import json
import sys

p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
tags = sys.argv[2:] or ['grasp_cup', 'lift_cup', 'pour', '捧杯', '前伸前',
                        '放回', '撤手', '悬垂']
a = json.load(open(p, encoding='utf-8'))['result'].get('arm') or {}
print('tags:', sorted(a))
for k in tags:
    v = a.get(k)
    if not v:
        print('%-10s EMPTY' % k)
        continue
    for s, st in sorted(v.items()):
        print('%-10s %s q=%s up=%+.2f el=%+.2f tw=%+.2f wp=%+.2f 外扩=%+.0f droop=%+.0f %s'
              % (k, s, [round(x, 2) for x in st['q']], st['up_roll'], st['q'][3],
                 st['twist'], st['q'][5], st['out_line_mm'], st['droop_mm'],
                 ','.join(st['bad']) or 'OK'))
