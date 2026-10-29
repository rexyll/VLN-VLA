#!/bin/bash
# gz_plugin.sh —— 由 start_scene.sh / start_demo.sh **source**, 不要直接执行
#
# 负责两件事:
#   1) 保证 gz 系统插件 `GraspCarry` 已编译(没有就现编)
#   2) export GZ_SIM_SYSTEM_PLUGIN_PATH, 让 `gz sim` 能通过 SDF 里的
#      <plugin filename="libGraspCarry.so" ...> 找到它
#
# 为什么插件不在 colcon 里: 它是纯 C++ 的 gz 插件, 没有 package.xml, 塞进
# ament_python 包既不合适也会让每次 colcon build 都重编。放在
# vla_ws/src/vla_follow/gz_plugins/ 用 cmake 单独构建最省事。
#
# 注意: 本脚本不能开 set -e / set -u —— 会被 /opt/ros/jazzy/setup.bash 的
#       AMENT_TRACE_SETUP_FILES 未定义打挂(见 start_scene.sh 顶部注释)。
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"

_PLUG_SRC="$VLA_HOME/vla_ws/src/vla_follow/gz_plugins"
_PLUG_SO="$_PLUG_SRC/build/libGraspCarry.so"

if [ ! -f "$_PLUG_SO" ]; then
    echo ">>> 首次编译 gz 插件 GraspCarry ..."
    mkdir -p "$VLA_HOME/vla_ws/logs"
    cmake -S "$_PLUG_SRC" -B "$_PLUG_SRC/build" -DCMAKE_BUILD_TYPE=Release \
        > "$VLA_HOME/vla_ws/logs/plugin_cmake.log" 2>&1
    cmake --build "$_PLUG_SRC/build" -j"$(nproc)" \
        >> "$VLA_HOME/vla_ws/logs/plugin_cmake.log" 2>&1
    if [ -f "$_PLUG_SO" ]; then
        echo ">>> GraspCarry 编译完成"
    else
        echo "!!! GraspCarry 编译失败, 见 logs/plugin_cmake.log"
        echo "    (抓取附着会失效: 壶/杯不会被手带着走)"
    fi
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$_PLUG_SRC/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
