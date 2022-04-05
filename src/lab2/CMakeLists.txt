cmake_minimum_required(VERSION 3.0.2)
project(lab2)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_control src/robot_control.cpp)
target_link_libraries(robot_control ${catkin_LIBRARIES})
