cmake_minimum_required(VERSION 3.0.2)
project(eskf_navi)

## 查找catkin依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  message_generation  
)

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

## 查找第三方库
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(tf2_ros REQUIRED)




add_message_files(
  FILES
  hedge_pos_a.msg
)



generate_messages(
  DEPENDENCIES
  std_msgs
)



## catkin包声明
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs tf2_ros
  DEPENDS Eigen3 GeographicLib yaml-cpp 
)

## 头文件搜索路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## 可执行文件
## add_executable(grid_map_node src/grid_map.cpp src/topology_map.cpp src/coordinate_converter.cpp)
add_executable(navigation_node src/navigation_node.cpp src/grid_map.cpp src/topology_map.cpp src/coordinate_converter.cpp)

## 链接库
# target_link_libraries(grid_map_node
#   ${catkin_LIBRARIES}
#   Eigen3::Eigen
#   Geographic
#   yaml-cpp
# )

target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  Geographic
  yaml-cpp
)

## 安装规则
# install(TARGETS grid_map_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 安装头文件（可选）
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)