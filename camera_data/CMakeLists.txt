cmake_minimum_required(VERSION 3.0.2)
project(camera_data)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Camera_Orientation.msg
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
    geometry_msgs
    visualization_msgs
    LIBRARIES drive_control
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  src/camera_publisher_1.py
  src/camera_publisher_2.py
  src/camera_subscriber.py
  src/aruco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
