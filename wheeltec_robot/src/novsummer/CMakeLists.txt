cmake_minimum_required(VERSION 3.0.2)
project(novsummer)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  move_base_msgs
  geometry_msgs
  tf
  amcl
  teb_local_planner
  costmap_converter
)

catkin_package(
  CATKIN_DEPENDS actionlib roscpp move_base_msgs geometry_msgs tf amcl teb_local_planner costmap_converter
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_to_goals src/move_to_goals.cpp)
target_include_directories(move_to_goals PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(move_to_goals ${catkin_LIBRARIES})
