from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.transport13 import Node

n = Node()
mv = Marker_V()

# 三条对照线: 高 z 粗线 / 低 z 粗线 / 贴地细线
cases = [
    (11, 0.35, 0.25, (0.0, 1.0, 0.0), -4.5),   # 绿 高处
    (12, 0.05, 0.25, (1.0, 0.0, 0.0), -3.5),   # 红 贴地
    (13, 0.02, 0.10, (0.0, 0.0, 1.0), -2.5),   # 蓝 贴地更细
]
for mid, z, w, col, yy in cases:
    mk = mv.marker.add()
    mk.ns = 'test'
    mk.id = mid
    mk.action = Marker.ADD_MODIFY
    mk.type = Marker.LINE_STRIP
    mk.scale.x = mk.scale.y = mk.scale.z = w
    mk.material.diffuse.r, mk.material.diffuse.g = col[0], col[1]
    mk.material.diffuse.b, mk.material.diffuse.a = col[2], 1.0
    mk.material.ambient.CopyFrom(mk.material.diffuse)
    for i in range(10):
        p = mk.point.add()
        p.x, p.y, p.z = -3.0 + i * 0.6, yy, z

print(n.request('/marker_array', mv, Marker_V, Boolean, 2000))

# 顺带加一个 POINTS 型对照
mv2 = Marker_V()
mk = mv2.marker.add()
mk.ns = 'test'
mk.id = 14
mk.action = Marker.ADD_MODIFY
mk.type = Marker.POINTS
mk.scale.x = mk.scale.y = mk.scale.z = 0.20
mk.material.diffuse.r, mk.material.diffuse.g = 1.0, 1.0
mk.material.diffuse.b, mk.material.diffuse.a = 0.0, 1.0
mk.material.ambient.CopyFrom(mk.material.diffuse)
for i in range(10):
    p = mk.point.add()
    p.x, p.y, p.z = -3.0 + i * 0.6, -1.5, 0.35
print(n.request('/marker_array', mv2, Marker_V, Boolean, 2000))
