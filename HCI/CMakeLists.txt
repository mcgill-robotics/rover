cmake_minimum_required(VERSION 2.8.3)
project(human_control_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
)

catkin_install_python(PROGRAMS 
  src/Controller/Joystick.py
  src/Controller/Gamepad.py
  src/pyqt/talker.py
  src/pyqt/listener.py
  src/pyqt/pyqt_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



