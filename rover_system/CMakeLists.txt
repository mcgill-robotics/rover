cmake_minimum_required(VERSION 3.5)
project(rover_system)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_libraries(human_control_interface)
ament_export_dependencies()
ament_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   arm_sim_manual.launch
   pilot_system.launch
   rover_robot.launch
   rover_sim_manual.launch
   # myfile2
   DESTINATION share/${PROJECT_NAME}
)