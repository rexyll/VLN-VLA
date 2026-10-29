#!/bin/bash
# tmp_m1_probe.sh —— 探测深度相机话题类型与数据
source /opt/ros/jazzy/setup.bash
for t in /depth_front /depth_front/depth_image /depth_front/points; do
  echo "=== $t ==="
  timeout 6 gz topic -i -t $t 2>/dev/null | head -6
done
echo "=== Python 订阅实测 ==="
python3 - <<'PYEOF'
import time
import gz.transport13 as gzt
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

node = gzt.Node()
got_img, got_pts = [], []
def icb(m):
    if not got_img: got_img.append(m)
def pcb(m):
    if not got_pts: got_pts.append(m)
node.subscribe(Image, '/depth_front', icb)
node.subscribe(Image, '/depth_front/depth_image', icb)
node.subscribe(PointCloudPacked, '/depth_front/points', pcb)
t0 = time.time()
while time.time()-t0 < 10 and not (got_img or got_pts):
    time.sleep(0.2)
if got_img:
    m = got_img[0]
    print('Image 到达: %dx%d fmt=%s bytes=%d' % (m.width, m.height, m.pixel_format_type, len(m.data)))
    if len(m.data) >= 4*100:
        import struct, math
        vals = struct.unpack('<%df' % (len(m.data)//4), m.data)
        fin = [v for v in vals if math.isfinite(v) and 0.05 < v < 12.0]
        print('按32FC1解析: 有限值=%d/%d' % (len(fin), len(vals)),
              ('min=%.2f max=%.2f' % (min(fin), max(fin))) if fin else '')
PYEOF
