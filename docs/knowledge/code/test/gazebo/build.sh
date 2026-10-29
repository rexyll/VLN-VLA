#!/usr/bin/env bash
# build.sh —— 编译 VLA 仿真桥插件（Gazebo Harmonic）
set -e
cd "$(dirname "$0")"
g++ -O2 -std=c++17 -fPIC -shared -o libvla_bridge.so vla_bridge.cpp \
    $(pkg-config --cflags --libs gz-sim8)
echo BUILD_OK
