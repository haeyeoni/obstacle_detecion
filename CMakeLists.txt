cmake_minimum_required(VERSION 3.1.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  pcl_conversions
  image_transport
  cv_bridge)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions( -march=native )

add_executable(obstacleDetection main.cc main.h)
target_link_libraries(obstacleDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
