cmake_minimum_required(VERSION 3.0.2)
project(leo_erc_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()

file(GLOB xacro_files worlds/*.world.xacro)
xacro_add_files(${xacro_files} TARGET generate_world_files)

install(DIRECTORY
  media
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.world"
  PATTERN "sdf" EXCLUDE
)