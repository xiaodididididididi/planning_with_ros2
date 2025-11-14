cmake_minimum_required(VERSION 3.31.3)
project(decision_center)

# set depend
set(DEPENDS
  "rclcpp"
)

# add lib
add_library(decision_center SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_center.cpp
)

set(target
  decision_center
)

# add depend
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  vehicle_info
)

# install lib,node and dictionary
install(TARGETS
  ${target}
  LIBRARY DESTINATION lib
)
