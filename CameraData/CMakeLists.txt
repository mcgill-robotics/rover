cmake_minimum_required(VERSION 3.0.2)
project(CameraData)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)



catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  sensor_msgs
  message_runtime
  LIBRARIES human_control_interface
)

catkin_install_python(PROGRAMS 
  src/camera_publisher_node.py
  src/camera_subscriber_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
