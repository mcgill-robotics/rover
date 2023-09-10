cmake_minimum_required(VERSION 3.5)
project(human_control_interface)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Joystick_input.msg
        msg/Gamepad_input.msg
    DEPENDENCIES
        
)

ament_export_libraries(human_control_interface)
ament_package()

catkin_install_python(PROGRAMS 
  src/Controller/Joystick.py
  src/Controller/Gamepad.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

