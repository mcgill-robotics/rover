cmake_minimum_required(VERSION 3.0.2)
project(autonomy)

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  std_msgs
  geometry_msgs
  gazebo_ros
  sensor_msgs
  visualization_msgs
  nav_msgs
)

# Declare catkin package
catkin_package()

# Install Python scripts (moving.py, mapping.py, and odom_republisher.py)
catkin_install_python(PROGRAMS
  scripts/moving.py
  scripts/mapping.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the autonomy_config.py module from the scripts directory
install(PROGRAMS
  scripts/autonomy_config.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/autonomy
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install RViz configurations
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
