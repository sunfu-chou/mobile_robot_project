cmake_minimum_required(VERSION 3.0.2)
project(lab1)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp 
  std_msgs
  geometry_msgs
  turtlesim
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(square_control src/square_control.cpp)
target_link_libraries(square_control ${catkin_LIBRARIES})
