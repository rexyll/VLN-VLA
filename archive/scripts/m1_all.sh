#!/bin/bash
# tmp_m1_all.sh —— 四向深度全检 + RTF
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, struct, math
import gz.transport13 as gzt
from gz.msgs10.image_pb2 import Image
from gz.msgs10.world_stats_pb2 import WorldStatistics

node = gzt.Node()
cams = {n: [] for n in ('front','left','right','back')}
def mk(nm):
    def cb(m):
        if not cams[nm]: cams[nm].append(m)
    return cb
for n in cams:
    node.subscribe(Image, '/depth_' + n, mk(n))
stats = []
def scb(m):
    if not stats: stats.append(m)
node.subscribe(WorldStatistics, '/stats', scb)
t0 = time.time()
while time.time()-t0 < 12 and not (all(cams.values()) and stats):
    time.sleep(0.2)
print('=== 四向深度 ===')
for n in ('front','left','right','back'):
    if cams[n]:
        m = cams[n][0]
        vals = struct.unpack('<%df' % (len(m.data)//4), m.data)
        fin = [v for v in vals if math.isfinite(v) and 0.05 < v < 12.0]
        if fin:
            import statistics
            print('%s: %dx%d 有效=%d min=%.2f 中位=%.2f max=%.2f' % (
                n, m.width, m.height, len(fin), min(fin), statistics.median(fin), max(fin)))
        else:
            print('%s: 无有效深度' % n)
    else:
        print(n, ': 未收到')
if stats:
    s = stats[0]
    print('=== RTF === sim_time=%.1fs real_time=%.1fs rtf=%.2f' % (
        s.sim_time.sec + s.sim_time.nsec*1e-9,
        s.real_time.sec + s.real_time.nsec*1e-9, s.real_time_factor))
PYEOF
