import sys
import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

node = gz_transport.Node()
got = []

def cb(msg):
    got.append(msg)

node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
import time
t0 = time.time()
while time.time() - t0 < 6 and not got:
    time.sleep(0.1)
if got:
    m = got[0]
    names = [p.name for p in m.pose]
    print('RECEIVED, models:', names[:8])
    for p in m.pose:
        if p.name in ('robot', 'person'):
            print(p.name, '%.3f %.3f yaw_stamp=%d' % (
                p.position.x, p.position.y, m.header.stamp.sec))
else:
    print('NO MESSAGE in 6s')
