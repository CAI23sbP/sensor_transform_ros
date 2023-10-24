cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)
project(sensor_transform_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  geometry_msgs
  std_msgs
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs pcl_ros sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sensor_transform_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}  ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
