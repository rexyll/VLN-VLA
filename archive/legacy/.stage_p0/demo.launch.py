#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo.launch.py —— 一键启动: Gazebo(带GUI) + ros_gz桥接 + 4个业务节点

用法:
  ros2 launch vla_follow demo.launch.py                     # 全自动演示
  ros2 launch vla_follow demo.launch.py auto_voice:=false   # 不自动发语音, 手动 send_voice.sh
  ros2 launch vla_follow demo.launch.py voice_delay:=8.0    # 调整语音触发时间
  ros2 launch vla_follow demo.launch.py start_gz:=false     # gz 已由外部启动
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')


def generate_launch_description():
    pkg_share = get_package_share_directory('vla_follow')
    world_file = os.path.join(pkg_share, 'worlds', 'work_area.sdf')
    bridge_cfg = os.path.join(pkg_share, 'config', 'bridge.yml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=world_file,
            description='Gazebo world 文件'),
        DeclareLaunchArgument(
            'auto_voice', default_value='true',
            description='是否自动播放并发送"请跟随我，保持2米的距离"'),
        DeclareLaunchArgument(
            'voice_delay', default_value='12.0',
            description='启动后多少秒发送语音指令'),
        DeclareLaunchArgument(
            'start_gz', default_value='true',
            description='是否由本 launch 拉起 gz sim。start_demo.sh 已自行启动 '
                        'gz, 必须传 start_gz:=false, 否则会起第二个 Gazebo '
                        '实例(抢同一批 gz topic, 仿真时钟错乱甚至直接退出)'),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(
                os.environ.get('VLA_HOME',
                               '/home/earl/workspace/VLA'),
                'vla_demo', 'vosk-model-small-cn-0.22'),
            description='Vosk 中文模型目录'),

        # Gazebo Harmonic: 服务端 + GUI (GUI 显示在桌面, 需 DISPLAY=:0)
        # render-engine ogre: VMware 虚拟显卡下 ogre2 渲染失败(白屏), 统一用 ogre1
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', '--render-engine', 'ogre',
                 LaunchConfiguration('world')],
            output='both',
            condition=IfCondition(LaunchConfiguration('start_gz')),
        ),

        # ROS2 <-> Gazebo 桥接
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             output='both',
             parameters=[{'config_file': bridge_cfg}]),

        # "人"的随机行走
        Node(package='vla_follow', executable='person_walk',
             output='both'),

        # 4 向深度相机感知层
        Node(package='vla_follow', executable='depth_percept',
             output='both'),

        # 相机识"人"(RGB暗色先验+深度对齐) → /perception/person
        Node(package='vla_follow', executable='person_vision',
             output='both'),

        # 真人语音识别 + 指令解析
        Node(package='vla_follow', executable='voice_node',
             output='both',
             parameters=[{'model_path': LaunchConfiguration('model_path')}]),

        # 跟随控制
        Node(package='vla_follow', executable='follower', output='both'),

        # 定时播放真人语音并触发识别
        Node(package='vla_follow', executable='voice_trigger',
             output='both',
             condition=IfCondition(LaunchConfiguration('auto_voice')),
             parameters=[
                 {'delay': LaunchConfiguration('voice_delay')},
             ]),

        # ---- P65/P71: Nav2 搜索段导航栈 (GOTO 由 NavigateToPose 接管) ----
        # 真值定位桥(GPU 激光无效, AMCL 不可用 → 仿真真值即定位)
        Node(package='vla_follow', executable='gt_tf',
             output='both', parameters=[{'use_sim_time': True}]),
        # 底盘速度复用器: 自研(对正/跟踪) / Nav2(搜索) 择路 + 硬闸
        Node(package='vla_follow', executable='cmd_mux',
             output='both', parameters=[{'use_sim_time': True}]),
        # 最小化导航栈(map_server+planner+controller+behavior+bt+smoother),
        # 由 lifecycle_manager 统一 configure/activate。
        # P71: 不再 include nav2_bringup/navigation_launch.py —— 它无条件拉起
        #      smoother/route/waypoint/docking/collision_monitor 五个我们用不到
        #      的节点, params 缺任一段就 Aborting bringup, /navigate_to_pose
        #      不存在 → GOTO 段全程冻结。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, 'launch', 'nav2_follow.launch.py')]),
            launch_arguments={
                'autostart': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
    ])
