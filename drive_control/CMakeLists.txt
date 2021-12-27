cmake_minimum_required(VERSION 3.0.2)
project(drive_control)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    message_generation
)


catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WheelSpeed.msg
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
    LIBRARIES drive_control
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  src/drive_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
