import sys
from gz.transport13 import Node
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.boolean_pb2 import Boolean

out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/gz_gui.png'
n = Node()
res = n.request('/gui/screenshot', StringMsg(data=out), StringMsg, Boolean, 5000)
print('res=', res)
