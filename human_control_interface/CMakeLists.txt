cmake_minimum_required(VERSION 2.8.3)
project(human_control_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

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
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  message_runtime
  LIBRARIES human_control_interface
)

catkin_install_python(PROGRAMS 
  src/Controller/Joystick.py
  src/Controller/Gamepad.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



