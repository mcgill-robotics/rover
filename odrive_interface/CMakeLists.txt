cmake_minimum_required(VERSION 3.0.2)
project(odrive_interface)



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
  Errors.msg
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
    LIBRARIES odrive_interface
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  src/ODrive.py
  src/odrive_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
