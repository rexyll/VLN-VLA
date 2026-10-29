#!/bin/bash
# tests/eval_repeat.sh <N> <tag> [dur] —— 同一配置重复 N 次评测, 输出中位数
#
# 位置: tests/   依赖: tests/eval_once.sh
# 背景: 单次评测的 in_band 波动可达 ±6pt(t24 42.5 vs t29 36.2, 同一份代码),
# 原因是被控对象自身的反馈把微小调度差异放大(混沌), 与"人"的回放无关。
# 因此任何 < 7pt 的差异都不足以判定优劣, 必须重复取样。
set +u
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
cd "$VLA_HOME" || exit 1
N=${1:-3}
TAG=${2:-rep}
DUR=${3:-120}
OUTDIR="$VLA_HOME/tests/out"

for i in $(seq 1 "$N"); do
    echo ">>> [$i/$N] $TAG"
    bash "$VLA_HOME/tests/eval_once.sh" "$DUR" "${TAG}_$i" rpl 2>&1 | tail -2
    sleep 3
done

python3 - "$TAG" "$N" "$OUTDIR" <<'PY'
import json, sys, statistics as st
tag, n, outdir = sys.argv[1], int(sys.argv[2]), sys.argv[3]
rows = []
for i in range(1, n + 1):
    f = '%s/%s_%d.json' % (outdir, tag, i)
    try:
        s = json.load(open(f, encoding='utf-8'))['summary']
    except Exception as e:
        print('%s 读取失败: %s' % (f, e))
        continue
    d, c = s['dist'], s.get('center', {})
    b = s.get('back', {})
    ct = s.get('contact', {})
    rows.append(dict(contact=ct.get('n', 0), ctev=ct.get('events', 0),
                     ctdmin=ct.get('dmin', 0) or 0,
                     in_band=d.get('in_band_pct', 0), near=d.get('near_pct', 0),
                     far=d.get('far_pct', 0), dmed=d.get('median', 0),
                     w10=c.get('within10_pct', 0), mx=c.get('max_deg', 0),
                     dmin=d.get('min_all', 0), lost=s['lost']['n'],
                     bk=b.get('pct', 0), bn=b.get('events', 0),
                     be=b.get('per_100s', 0)))
if not rows:
    sys.exit('无数据')
keys = ['in_band', 'near', 'far', 'dmed', 'w10', 'mx', 'dmin',
        'bk', 'bn', 'be', 'contact', 'ctev', 'ctdmin']
print('\n=== %s  n=%d ===' % (tag, len(rows)))
print('%-8s %8s %8s %8s' % ('指标', '中位', '最小', '最大'))
for k in keys:
    v = [r[k] for r in rows]
    print('%-8s %8.2f %8.2f %8.2f' % (k, st.median(v), min(v), max(v)))
print('lost: %s' % [r['lost'] for r in rows])
print('逐次 in_band: %s' % [round(r['in_band'], 1) for r in rows])
PY
