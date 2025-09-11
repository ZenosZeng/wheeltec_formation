cmake_minimum_required(VERSION 3.0.2)
project(tester)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
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

add_executable(line_follow src/line_follow.cc)
add_executable(line_move src/line_move.cc)
add_executable(line_move_2d src/line_move_2d.cc)

target_link_libraries(line_follow ${catkin_LIBRARIES})
target_link_libraries(line_move ${catkin_LIBRARIES})
target_link_libraries(line_move_2d ${catkin_LIBRARIES})

add_executable(listener_node src/listener_node.cc)
target_link_libraries(listener_node ${catkin_LIBRARIES})

add_executable(line_follow_2d src/line_follow_2d.cc)
target_link_libraries(line_follow_2d ${catkin_LIBRARIES})