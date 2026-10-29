# gz_plugins —— 随仿真一起加载的 gz-sim C++ 系统插件
#
# 不是 colcon 包(没有 package.xml), 由 tools/start_scene.sh 按需 cmake 构建,
# 产物 libGraspCarry.so 通过 GZ_SIM_SYSTEM_PLUGIN_PATH 被 gz sim 找到。
# 这样既不用把 C++ 混进 ament_python 包, 也不用每次 colcon build 都重编。
#
# 手工构建:
#   cd vla_ws/src/vla_follow/gz_plugins && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
cmake_minimum_required(VERSION 3.16)
project(vla_gz_plugins LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
# 不要 COMPONENTS gz: 那个组件会把 gz-gui8 拉进来(gz-sim8-gz 依赖 Qt),
# 本机没装 GUI 的开发头, 一找就 NOT FOUND。插件只需要核心 sim8 库。
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-math7 REQUIRED)

add_library(GraspCarry SHARED GraspCarry.cc)
target_link_libraries(GraspCarry
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
  gz-math7::gz-math7
)
set_target_properties(GraspCarry PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
