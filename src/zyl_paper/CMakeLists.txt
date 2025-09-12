cmake_minimum_required(VERSION 3.0.2)
project(zyl_paper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zyl_control
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(leader_controller src/leader_controller.cc)
target_link_libraries(leader_controller ${catkin_LIBRARIES})

add_executable(rp_controller src/rp_controller.cc)
target_link_libraries(rp_controller ${catkin_LIBRARIES})

add_executable(rigid_controller src/rigid_controller.cc)
target_link_libraries(rigid_controller ${catkin_LIBRARIES})