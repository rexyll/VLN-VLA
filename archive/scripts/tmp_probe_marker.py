import gz.msgs10.marker_pb2 as m

mk = m.Marker()
print("Marker fields:", [f.name for f in mk.DESCRIPTOR.fields])
print("material type:", type(mk.material))
print("material fields:", [f.name for f in mk.material.DESCRIPTOR.fields])
print("ambient type:", type(mk.material.ambient))
print("ambient fields:", [f.name for f in mk.material.ambient.DESCRIPTOR.fields])
print("pose fields:", [f.name for f in mk.pose.DESCRIPTOR.fields])
print("position fields:", [f.name for f in mk.pose.position.DESCRIPTOR.fields])
print("enum:")
for v in mk.DESCRIPTOR.enum_types[0].values:
    print("  ", v.name, v.number)
