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
  MotorError.msg
  MotorState.msg
  ODriveStatus.msg
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
  src/node_odrive_arm.py
  src/node_odrive_drive.py
  src/node_odrive_comb.py
  src/node_brushed.py
  src/node_control_gui.py
  src/ros_path_config.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
