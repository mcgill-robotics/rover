cmake_minimum_required(VERSION 3.5)
project(arm_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

## Generate messages in the 'msg' folder

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ArmControllerInput.msg
    DEPENDENCIES
        
)

ament_export_libraries(arm_control)
ament_export_dependencies(rclcpp rclpy std_msgs)
ament_package()

catkin_install_python(PROGRAMS
  src/arm_controller.py
  sim/arm_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
