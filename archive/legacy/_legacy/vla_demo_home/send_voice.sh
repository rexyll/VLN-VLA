#!/bin/bash
# send_voice.sh [wav文件|mic] —— 播放真人语音指令并送入识别节点
# 项目根: $VLA_HOME (默认 /home/earl/workspace/VLA = Windows 端 Z:\VLA)
# 例: bash send_voice.sh                                    # 默认"请跟随我"(距离用配置默认1m)
#     bash send_voice.sh $VLA_HOME/vla_demo/audio/stop_follow.wav   # "请停止跟随"
#     bash send_voice.sh $VLA_HOME/vla_demo/audio/follow_2m.wav     # "保持2米的距离"(显式覆盖)
#     bash send_voice.sh mic                                # 从麦克风识别 15 秒
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
source /opt/ros/jazzy/setup.bash
PKG_AUDIO="$VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/audio"
WAV="${1:-$PKG_AUDIO/follow_me.wav}"

# -w 1: 等待订阅匹配后再发布, 避免消息丢失; 双发规避 DDS 发现竞态
if [ "$WAV" = "mic" ]; then
    ros2 topic pub -1 -w 1 /voice/request std_msgs/msg/String "{data: 'mic'}" >/dev/null
    echo ">>> 已请求麦克风识别"
    exit 0
fi
if [ ! -f "$WAV" ]; then
    echo "音频不存在: $WAV"
    exit 1
fi
aplay "$WAV" >/dev/null 2>&1 &
timeout 10 ros2 topic pub -1 -w 1 /voice/request std_msgs/msg/String "{data: '$WAV'}" >/dev/null 2>&1
sleep 1.5
timeout 10 ros2 topic pub -1 -w 1 /voice/request std_msgs/msg/String "{data: '$WAV'}" >/dev/null 2>&1
echo ">>> 已发送语音指令音频(双发): $WAV (识别结果见日志窗口)"
