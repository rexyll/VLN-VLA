#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav2_follow.launch.py —— 最小化 Nav2 导航栈 (P71)

为什么不用 nav2_bringup/navigation_launch.py:
  1) Jazzy 的官方 launch 无条件拉起 10 个节点
     (controller/smoother/planner/route/behavior/velocity_smoother/
      collision_monitor/bt_navigator/waypoint_follower/docking_server),
     并把它们全部塞进 lifecycle_manager 的 node_names。只要 params 里
     少一个节点的完整段落, 该节点 configure 就抛
     "parameter 'observation_sources' is not initialized" 之类的异常,
     lifecycle_manager 直接 "Aborting bringup" —— 结果是
     /navigate_to_pose action 根本不存在, GOTO 段全程冻结。
  2) 我们真正需要的只有 6 个: map_server + planner + controller +
     behavior + bt_navigator + velocity_smoother。
     smoother/route/waypoint/docking 这套是给"多点巡游/自动充电"用的,
     本工程不用, 却要为它们各写一段参数, 多一处出错机会。
  3) collision_monitor 在本机是天然空转: 它的观测源只有激光(P2 已确认
     GPU 激光在本渲染栈输出无效), 没有观测源就既不能 stop 也不能
     slowdown, 留着只是多一个能让 bringup 失败的点。真正的近距离硬闸
     在 cmd_mux(4 向深度 < 0.30m 清零), 且它是最终闸门、不被 Nav2 绕过。

速度链路(与官方一致, 只是末端接 cmd_mux):
  controller_server --cmd_vel_nav--> velocity_smoother
    --/cmd_vel_smoothed--> cmd_mux --/robot/cmd_vel--> gz 差速插件
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'map_server',
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description():
    pkg_share = get_package_share_directory('vla_follow')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Nav2 参数文件')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='是否自动 configure+activate')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    sim = {'use_sim_time': True}

    common = dict(output='both', parameters=[params_file, sim])

    nodes = [
        # 静态地图(gt_tf 提供真值定位, 不用 AMCL)
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', **common),
        # 全局规划 + 局部控制
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server',
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        # 恢复行为(spin/backup/wait), 卡住时由 BT 调用
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server',
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        # 行为树导航器 → 提供 /navigate_to_pose action
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **common),
        # 速度平滑: 订阅 cmd_vel_nav, 输出 /cmd_vel_smoothed
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        # 生命周期管理: 显式列出 6 个节点, 与上面严格一一对应
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='both',
             parameters=[params_file, sim, {
                 'autostart': autostart,
                 'node_names': LIFECYCLE_NODES,
                 'bond_timeout': 8.0,
             }]),
    ]

    return LaunchDescription([declare_params, declare_autostart] + nodes)
