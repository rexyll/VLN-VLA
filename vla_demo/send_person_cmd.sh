#!/bin/bash
# send_person_cmd.sh [drink|walk|stop] —— 给场景里的"人"下发指令
#
# 用法:
#   bash vla_demo/send_person_cmd.sh drink    # 人走到桌边 → 说"我要喝水" → 等机器人递水
#   bash vla_demo/send_person_cmd.sh walk     # 取消喝水, 回到随机行走
#   bash vla_demo/send_person_cmd.sh stop     # 就地停住
#
# 链路(为什么是这两步):
#   ① /person/command {"action":"GO_DRINK"}
#      → person_walk 进 GOTO 状态, 走到 scene_layout.PERSON_SPOT 站定、面向机器人
#   ② person_walk 到位后自己发 /voice/request "text:我要喝水"
#      → voice_node 解析成 {"action":"DRINK"} 发 /robot/command
#      → follower 停车让权 + drink_task 执行整套任务
#   也就是说**只需要这一条命令**, 后面的语音是"人"说的, 不是我们代说的。
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
source /opt/ros/jazzy/setup.bash

CMD="${1:-drink}"
case "$CMD" in
  drink) ACTION="GO_DRINK" ;;
  walk)  ACTION="DRINK_DONE" ;;
  stop)  ACTION="STOP" ;;
  *) echo "用法: $0 [drink|walk|stop]"; exit 1 ;;
esac

# -w 1 等订阅匹配; 双发规避 DDS 发现竞态(与 send_voice.sh 一致)
for i in 1 2; do
  timeout 10 ros2 topic pub -1 -w 1 /person/command std_msgs/msg/String \
      "{data: '{\"action\": \"$ACTION\"}'}" >/dev/null 2>&1
  sleep 0.6
done
echo ">>> 已给\"人\"下发: $CMD ($ACTION)"
if [ "$CMD" = "drink" ]; then
  echo ">>> 接下来应该看到: 人走向桌边(2.17, 1.17) → 面向机器人 → 语音日志出现\"我要喝水\" → 机器人开始倒水"
fi
