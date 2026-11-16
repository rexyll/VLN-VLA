cmake_minimum_required(VERSION 3.16)
project(mca_drink_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(mca_drink_core INTERFACE)
target_compile_features(mca_drink_core INTERFACE cxx_std_17)
target_include_directories(mca_drink_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/MotionControlAlgorithms/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/mca_upstream>)

if(BUILD_TESTING)
  ament_add_gtest(test_mca_drink_core test/test_mca_drink_core.cpp)
  target_link_libraries(test_mca_drink_core mca_drink_core)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/MotionControlAlgorithms/include/ DESTINATION include/mca_upstream)
install(FILES third_party/MotionControlAlgorithms/LICENSE
              third_party/MotionControlAlgorithms/NOTICE
        DESTINATION share/${PROJECT_NAME}/third_party/MotionControlAlgorithms)

ament_export_include_directories(include include/mca_upstream)
ament_package()
