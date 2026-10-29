#!/bin/bash
# tmp_m1_png.sh —— 把 front 深度帧渲染成 PNG(0-12m→灰度) 以判定行序
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, struct
import numpy as np
import gz.transport13 as gzt
from gz.msgs10.image_pb2 import Image
node = gzt.Node()
got = []
def cb(m):
    if not got: got.append(m)
node.subscribe(Image, '/depth_left', cb)
t0 = time.time()
while time.time()-t0 < 8 and not got:
    time.sleep(0.1)
assert got, 'no image'
m = got[0]
a = np.frombuffer(m.data[:m.width*m.height*4], dtype='<f4').reshape(m.height, m.width)
# 灰度: 0m→黑, 12m→白, inf→红(标记用255,255,255? 简化为黑)
g = np.clip(a / 12.0, 0, 1)
g[~np.isfinite(a)] = 0.0
rgb = (g * 255).astype(np.uint8)
png = np.repeat(rgb[:, :, None], 3, axis=2)
# 加行号刻度: 每隔40行画一条白线
png[::40, :, :] = 255
with open('/home/earl/depth_left.raw', 'wb') as f:
    f.write(png.tobytes())
print('saved %dx%d' % (m.width, m.height))
PYEOF
ffmpeg -y -loglevel error -f rawvideo -pixel_format rgb24 -video_size 320x240 -i ~/depth_left.raw -frames:v 1 ~/depth_left.png && echo PNG-OK
