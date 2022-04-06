cmake_minimum_required(VERSION 3.0.2)
project(lab3)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp 
  std_msgs
  nav_msgs
  std_srvs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(a_star src/a_star.cpp)
target_link_libraries(a_star ${catkin_LIBRARIES})

add_executable(lab3 src/main.cpp)
target_link_libraries(lab3 a_star)
