cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
  FILES
  Joystick_input.msg
  Gamepad_input.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES custom_msgs
  CATKIN_DEPENDS message_runtime
)

