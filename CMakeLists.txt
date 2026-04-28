cmake_minimum_required(VERSION 3.8)
project(serial_joy_receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cango_msgs REQUIRED)

add_executable(serial_joystick_receiver src/serial_joystick_receiver.cpp)
add_executable(serial_trigger_sender src/serial_trigger_sender.cpp)
add_executable(joy_to_robot_control src/joy_to_robot_control.cpp)

ament_target_dependencies(serial_joystick_receiver
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(serial_trigger_sender
  rclcpp
  std_msgs
  cango_msgs
)

ament_target_dependencies(joy_to_robot_control
  rclcpp
  sensor_msgs
  cango_msgs
)

install(TARGETS
  serial_joystick_receiver
  serial_trigger_sender
  joy_to_robot_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()