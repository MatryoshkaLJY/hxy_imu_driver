# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20)
project(hxy_imu_driver LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This minimal serial client currently targets Linux.")
endif()

option(IMU_BUILD_ROS2 "Build the ROS 2 node and ament package" ON)
get_filename_component(IMU_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(MAVLINK_INCLUDE_DIR "${IMU_SOURCE_DIR}/third_party/mavlink"
    CACHE PATH "Directory containing the pinned MAVLink common/mavlink.h")
if(NOT EXISTS "${MAVLINK_INCLUDE_DIR}/common/mavlink.h")
  message(FATAL_ERROR "Missing vendored MAVLink headers: keep third_party/mavlink or set MAVLINK_INCLUDE_DIR.")
endif()

function(imu_target target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} SYSTEM PRIVATE "${MAVLINK_INCLUDE_DIR}")
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()

add_executable(imu_monitor imu_monitor.cpp)
imu_target(imu_monitor)

if(IMU_BUILD_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  add_executable(imu_node imu_node.cpp)
  imu_target(imu_node)
  ament_target_dependencies(imu_node rclcpp rcl_interfaces sensor_msgs)
  install(TARGETS imu_node imu_monitor DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
          PATTERN "__pycache__" EXCLUDE)
  install(FILES README.rst LICENSE DESTINATION share/${PROJECT_NAME})
  install(FILES "${MAVLINK_INCLUDE_DIR}/COPYING" "${MAVLINK_INCLUDE_DIR}/README.vendor"
          DESTINATION share/${PROJECT_NAME}/mavlink)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_executable(imu_test_frames test_frames.cpp)
  imu_target(imu_test_frames)
  add_executable(imu_protocol_test test_protocol.cpp)
  imu_target(imu_protocol_test)
  target_compile_options(imu_protocol_test PRIVATE -UNDEBUG)
  add_test(NAME imu_protocol COMMAND imu_protocol_test)
  add_test(NAME monitor_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_monitor.py
            $<TARGET_FILE:imu_monitor> $<TARGET_FILE:imu_test_frames>)
  set_tests_properties(monitor_integration PROPERTIES TIMEOUT 15)
  if(IMU_BUILD_ROS2)
    find_package(ament_cmake_test REQUIRED)
    ament_add_test(ros2_integration
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_ros2.py
              $<TARGET_FILE:imu_node> $<TARGET_FILE:imu_test_frames>
              ${CMAKE_CURRENT_SOURCE_DIR}/config/imu.yaml
      TIMEOUT 60)
  endif()
endif()

if(IMU_BUILD_ROS2)
  ament_package()
endif()
