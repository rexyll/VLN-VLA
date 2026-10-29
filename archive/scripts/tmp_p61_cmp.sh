#!/bin/bash
# P61: 差速轮方向约定的纯净对照实验
# 独立起 gz sim(server only, 无 ROS/follower 干扰), 跑判定脚本, 再换下一个世界。
# 用法: bash p61_cmp.sh <world1> <tag1> <world2> <tag2> ...
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash 2>/dev/null

run_one () {
  local world="$1"; local tag="$2"
  echo "############################################################"
  echo "### WORLD: $world   TAG: $tag"
  echo "############################################################"
  setsid nohup gz sim -s -r -v 2 "$world" > "/tmp/gz_$tag.log" 2>&1 < /dev/null &
  local gp=$!
  sleep 18
  cd /home/earl/workspace/VLA
  timeout 90 python3 p61_axis_check.py "$tag" 2>&1 | grep -E '初始|cmd\.|VERDICT|->|差速轮'
  # 精确清理本轮 gz 进程
  pkill -9 -f "gz sim -s -r -v 2 $world" 2>/dev/null
  pkill -9 -f 'gz sim serve[r]' 2>/dev/null
  sleep 4
  echo
}

cd /home/earl/workspace/VLA || exit 1
while [ $# -ge 2 ]; do
  run_one "$1" "$2"
  shift 2
done
echo "ALL DONE"
