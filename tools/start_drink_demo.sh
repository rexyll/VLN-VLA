#!/bin/bash
# start_drink_demo.sh —— 轻量起"喝水演示"的业务节点
#
#   看喝水的完整链路(人走过去说话 → 机器人倒水递水):
#       bash tools/start_scene.sh            # 世界 + 桥接
#       bash tools/start_drink_demo.sh       # 本脚本: 人/语音/喝水任务
#       bash vla_demo/send_person_cmd.sh drink
#
# 与 demo.launch.py 的区别: 不起可视化外围。
#
# P97(§6.7 第 4 项): 把 Nav2 接回来 —— 喝水演示里人会被随机行走带得很远,
# follower 视觉丢失后的 GOTO 搜索段由 NavigateToPose 接管, 能绕过立柱/
# 桌子跨场地找回人; 不起 Nav2 时 follower 只能就地扫描等自己转回来。
#   NAV2=0 bash tools/start_drink_demo.sh   # 退回旧的不带 Nav2 行为
# 不怕 Nav2 抢底盘: controller_server 只往 /cmd_vel_nav→/cmd_vel_smoothed
# 写, 唯一写 /robot/cmd_vel 的仍是 cmd_mux, 由 /nav2_active 择路(仅
# follower 的 GOTO 段置真); 喝水期间 cmd_mux 另有 300s 输入锁兜底。
#
# P93 起**必须带上 follower 这一组**(感知 → follower → cmd_mux): 喝水任务
# 收尾后要把底盘交还给它, 让人喝完走开时机器人能重新跟上去。缺任何一个
# 都会变成"交了权但没人接": 没有 cmd_mux 时 follower 的 /cmd_vel_follow
# 根本到不了 /robot/cmd_vel; 没有感知时 follower 不知道人在哪。
# Nav2 还是不起 —— follower 的 Nav2 action server 不可达时会自己降级成

# 日志: vla_ws/logs/{pw,voice,drink,percept,vision,mux,follower,
#                   gt_tf,viz,nav2}.log
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
LOG="$VLA_HOME/vla_ws/logs"
mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"
NAV2="${NAV2:-1}"

# 先清掉上一轮的残留(按 cmdline 精确匹配, 别用 -x —— 进程名全是 python3)
for pat in 'vla_follow/person_walk' 'vla_follow/voice_node' \
           'vla_follow/drink_task' 'vla_follow/follower' \
           'vla_follow/cmd_mux' 'vla_follow/depth_percept' \
           'vla_follow/person_vision' 'vla_follow/gt_tf' \
           'vla_follow/gz_viz' 'nav2_map_server' 'nav2_controller' \
           'nav2_planner' 'nav2_behaviors' 'nav2_bt_navigator' \
           'nav2_velocity_smoother' 'nav2_lifecycle_manager' \
           'nav2_follow.launch'; do
    pkill -f "$pat" 2>/dev/null
done
sleep 1

launch() {   # launch <日志名> <命令...>
    local n="$1"; shift
    setsid nohup "$@" > "$LOG/$n.log" 2>&1 < /dev/null &
}

S="--ros-args -p use_sim_time:=true"
# 感知两个节点必须在 follower 之前起: follower 要订阅它们的话题
launch percept ros2 run vla_follow depth_percept $S
launch vision ros2 run vla_follow person_vision $S
launch mux ros2 run vla_follow cmd_mux $S
launch follower ros2 run vla_follow follower $S
if [ "$NAV2" = "1" ]; then
    # P97: 跨场地导航 —— gt_tf 必须先于 Nav2(costmap/控制器都要
    # map→odom→base_link TF); gz_viz 把 Nav2 规划路径画进 gz GUI
    launch gt_tf ros2 run vla_follow gt_tf $S
    launch nav2 ros2 launch vla_follow nav2_follow.launch.py
    launch viz ros2 run vla_follow gz_viz $S
fi
launch pw ros2 run vla_follow person_walk $S
launch voice ros2 run vla_follow voice_node
launch drink ros2 run vla_follow drink_task $S

sleep 8
for n in percept vision mux follower gt_tf nav2 viz pw voice drink; do
    printf '>>> %-8s %s\n' "$n" "$(tail -1 "$LOG/$n.log" 2>/dev/null | cut -c1-100)"
done
if [ "$NAV2" = "1" ]; then
    echo ">>> 已起 感知/follower/cmd_mux/person_walk/voice_node/drink_task + gt_tf/Nav2/gz_viz"
else
    echo ">>> 已起 感知/follower/cmd_mux/person_walk/voice_node/drink_task (NAV2=0, 无导航)"
fi
echo ">>> 下一步: bash vla_demo/send_person_cmd.sh drink"
