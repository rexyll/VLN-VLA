#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""voice_trigger.py —— 演示时序控制

启动 delay 秒后: 用 aplay 播放真人语音(若声卡可用), 同时把音频路径
发到 /voice/request 触发识别 —— 模拟"人说出指令"。
"""
import os
import subprocess

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String


class VoiceTrigger(Node):

    def __init__(self):
        super().__init__('voice_trigger')
        # P79: 默认语音从 follow_2m.wav("保持两米") 换成 follow_me.wav
        # ("请跟随我" —— 不带距离)。跟随区间已改为 0.5~1.5m, 演示语音不该
        # 把目标距离钉在 2.0m(那會让距离环长期偏近侧饱和); 不带距离时
        # follower 用 vision_config.json 的 track_dist_m(1.0) 作为默认。
        default_wav = os.path.join(
            get_package_share_directory('vla_follow'), 'audio',
            'follow_me.wav')
        self.declare_parameter('wav', default_wav)
        self.declare_parameter('delay', 12.0)
        self.declare_parameter('play_audio', True)
        self.pub = self.create_publisher(String, '/voice/request', 10)
        delay = float(self.get_parameter('delay').value)
        self.timer = self.create_timer(delay, self._fire)
        wav = self.get_parameter('wav').value
        self.get_logger().info(
            f'TRIGGER {delay:.0f} 秒后将播放语音指令: {wav}')

    def _fire(self):
        self.timer.cancel()
        wav = self.get_parameter('wav').value
        self.get_logger().info(
            'TRIGGER ★ 播放真人语音指令: "请跟随我"(距离由配置决定 1.0m)')
        if self.get_parameter('play_audio').value:
            subprocess.Popen(
                ['aplay', wav],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        msg = String()
        msg.data = wav
        self.pub.publish(msg)
        self.get_logger().info('TRIGGER 已将音频送入语音识别节点')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
