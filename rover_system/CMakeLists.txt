cmake_minimum_required(VERSION 3.0.2)
project(rover_system)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  LIBRARIES human_control_interface
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   arm_sim_manual.launch
   pilot_system.launch
   rover_robot.launch
   rover_sim_manual.launch
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)