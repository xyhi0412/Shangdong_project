cmake_minimum_required(VERSION 3.0.2)
project(obstacle_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/obstacle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

