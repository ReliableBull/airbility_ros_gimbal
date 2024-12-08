cmake_minimum_required(VERSION 3.0.2)
project(gimbal_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gimbal_angle src/gimbal_angle.cpp)
target_link_libraries(gimbal_angle ${catkin_LIBRARIES})
