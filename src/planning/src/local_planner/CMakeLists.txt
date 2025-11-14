cmake_minimum_required(VERSION 3.31.3)
project(local_planner)

# set depends
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
  "tf2"
)

# add lib
add_library(local_planner SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/local_trajectory_combiner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_smoother.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_smoother.cpp
)

set(target
  local_planner
)

# add depends
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  mathlibs
  vehicle_info
  decision_center
)

# install lib,node and dictionary
install(TARGETS
  ${target}
  LIBRARY DESTINATION lib
)
