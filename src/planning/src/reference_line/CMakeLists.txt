cmake_minimum_required(VERSION 3.31.3)
project(reference_line)

# set depends
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
)

# add lib
add_library(reference_line SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_creator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_smoother.cpp
)

set(target
  reference_line
)

# add depends
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  mathlibs
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

# install lib,node and dictionary
install(TARGETS
  ${target}
  LIBRARY DESTINATION lib
)
