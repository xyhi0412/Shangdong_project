cmake_minimum_required(VERSION 3.0.2)
project(road_seg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ros_seg_infer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

