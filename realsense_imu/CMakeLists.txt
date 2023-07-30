cmake_minimum_required(VERSION 2.8.3)
project(realsense_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Realsense_imu_data.msg
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
  LIBRARIES realsense_imu
)

catkin_install_python(PROGRAMS 
  src/realsense_imu_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



