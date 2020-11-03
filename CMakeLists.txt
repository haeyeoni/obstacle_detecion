cmake_minimum_required(VERSION 3.1.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  pcl_conversions)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs tf pcl_conversions
  DEPENDS EIGEN3 PCL
  
)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions( -march=native )

add_executable(obstacleDetection main.cc)
target_link_libraries(obstacleDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES})