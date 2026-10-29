import io
import re
import collections

L = '/home/earl/workspace/VLA/vla_ws/logs/demo.log'
TRK = re.compile(r'FOLLOWER \[([^|]+)\|')
DST = re.compile(r'距离=([\d.]+)m')
CMD = re.compile(r'指令v=([+-][\d.]+)')
VR = re.compile(r'vr=([+-][\d.]+) pv=([+-][\d.]+)')
NOTE = re.compile(r'w=[+-][\d.]+ (.*?) \|ff=')

lines = [l for l in io.open(L, encoding='utf-8', errors='replace')
         if 'FOLLOWER [' in l]
print('总行 %d' % len(lines))
print('模式分布:', collections.Counter(m.group(1) for m in
                                 (TRK.search(l) for l in lines) if m))

far, band = [], []
for l in lines:
    m = TRK.search(l)
    if not m or m.group(1) != '跟踪':
        continue
    d = DST.search(l)
    if not d:
        continue
    d = float(d.group(1))
    if d > 1.5:
        far.append(l)
    elif 0.9 <= d <= 1.1:
        band.append(l)


def show(rows, n=8):
    for l in rows[-n:]:
        s = l.split('FOLLOWER', 1)[1]
        print('  ' + s.strip()[:230])


print('\n--- 距离>1.5m 的跟踪行 ---')
show(far)
print('\n--- 距离 0.9~1.1m 的跟踪行 ---')
show(band, 6)

# 前馈质量: pv 与真实所需速度(用相邻 log 的 距离差 估算太粗, 只看分布)
pv = [float(VR.search(l).group(2)) for l in lines if VR.search(l)]
if pv:
    pv.sort()
    print('\npv(人速估计) 样本 %d: 中位 %+.2f  p10 %+.2f  p90 %+.2f'
          % (len(pv), pv[len(pv) // 2], pv[int(.1 * len(pv))],
             pv[int(.9 * len(pv))]))

notes = collections.Counter()
for l in lines:
    m = NOTE.search(l)
    if m and m.group(1).strip():
        notes[m.group(1).split()[0]] += 1
print('\n安全层 note 分布:', notes.most_common(8))
