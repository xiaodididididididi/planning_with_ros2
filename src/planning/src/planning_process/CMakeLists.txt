cmake_minimum_required(VERSION 3.31.3)
project(planning_process)

# set depends
set(DEPENDS
  "rclcpp"
  "nav_msgs"
  "base_msgs"
  "tf2"
  "tf2_ros"
)

# add lib
add_executable(planning_process
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp
)

set(target
  planning_process
)

# add depends
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  decision_center
  vehicle_info
  reference_line
  local_planner
)

# install lib,node and dictionary
install(TARGETS
  ${target}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)
