#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p76_probe.py —— 验证 gz.transport13 Python 端能创建/移动/删除静态可视模型
签名: Node.request(service, request, request_type, response_type, timeout_ms)
"""
import sys

import gz.transport13 as gz_transport
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose

W = '/world/follow_world'
n = gz_transport.Node()


def call(name, req, rqt):
    try:
        return n.request(W + name, req, rqt, Boolean, 3000)
    except Exception as e:
        return ('EXC', repr(e))


SDF = ('<?xml version="1.0"?><sdf version="1.9"><model name="viz_test">'
       '<static>true</static><pose>2 2 0.03 0 0 0</pose>'
       '<link name="l"><visual name="v">'
       '<geometry><sphere><radius>0.15</radius></sphere></geometry>'
       '<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>'
       '</visual></link></model></sdf>')

ef = EntityFactory()
ef.sdf = SDF
ef.name = 'viz_test'
print('create   ->', call('/create', ef, EntityFactory))

p = Pose()
p.name = 'viz_test'
p.position.x = 3.0
p.position.y = 0.5
p.position.z = 0.03
print('set_pose ->', call('/set_pose', p, Pose))

rm = EntityFactory()
rm.name = 'viz_test'
rm.type = 1  # MODEL
print('remove   ->', call('/remove', rm, EntityFactory))
sys.exit(0)
