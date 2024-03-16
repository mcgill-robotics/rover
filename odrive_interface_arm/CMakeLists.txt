cmake_minimum_required(VERSION 3.0.2)
project(odrive_interface_arm)



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
  MotorError.msg
  MotorState.msg
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
    LIBRARIES odrive_interface_arm
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  src/node_odrive.py
  src/node_brushed_test.py
  src/node_arm_viz.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
