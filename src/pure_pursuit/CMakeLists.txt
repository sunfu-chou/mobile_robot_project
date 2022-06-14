cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp 
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit src/my.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
