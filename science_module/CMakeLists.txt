cmake_minimum_required(VERSION 3.0.2)
project(science_module)


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

catkin_package(
    CATKIN_DEPENDS 
    roscpp
    rospy 
    std_msgs
    LIBRARIES science_module
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  sim/science_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
