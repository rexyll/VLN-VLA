cmake_minimum_required(VERSION 3.16)
project(mca_drink_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(MCA_GENERATED_ARM_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_arm_models)
file(MAKE_DIRECTORY ${MCA_GENERATED_ARM_DIR})
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/export_arm_urdf.py
          --output-dir ${MCA_GENERATED_ARM_DIR}
  RESULT_VARIABLE MCA_ARM_MODEL_GENERATION_RESULT
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT MCA_ARM_MODEL_GENERATION_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to generate MCA arm URDF models from vla_follow.arm_kin")
endif()

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
  target_compile_definitions(test_mca_drink_core PRIVATE
    MCA_RIGHT_ARM_URDF_PATH="${MCA_GENERATED_ARM_DIR}/drink_arm_R.urdf"
    MCA_LEFT_ARM_URDF_PATH="${MCA_GENERATED_ARM_DIR}/drink_arm_L.urdf"
    MCA_RIGHT_ARM_FK_PATH="${MCA_GENERATED_ARM_DIR}/drink_arm_R_fk_reference.txt"
    MCA_LEFT_ARM_FK_PATH="${MCA_GENERATED_ARM_DIR}/drink_arm_L_fk_reference.txt")
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/MotionControlAlgorithms/include/ DESTINATION include/mca_upstream)
install(FILES third_party/MotionControlAlgorithms/LICENSE
              third_party/MotionControlAlgorithms/NOTICE
        DESTINATION share/${PROJECT_NAME}/third_party/MotionControlAlgorithms)

ament_export_include_directories(include include/mca_upstream)
ament_package()
