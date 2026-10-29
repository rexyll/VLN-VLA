from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
import time

res = {}


def cb(m):
    for p in m.pose:
        res[p.name] = (p.position.x, p.position.y, p.position.z,
                       p.orientation.w, p.orientation.z)


n = Node()
n.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
time.sleep(3)
print('实体数:', len(res))
for k in sorted(res):
    v = res[k]
    print('%-28s x=%7.3f y=%7.3f z=%6.3f w=%6.3f zq=%6.3f' % ((k,) + v))
