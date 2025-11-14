cmake_minimum_required(VERSION 3.31.3)
project(vehicle_info)

# set depends
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
  "tf2"
)

# add lib
add_library(vehicle_info SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/main_car/main_car_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/obs_car/obs_car_info.cpp
)

set(target
  vehicle_info
)

# add depends
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  mathlibs
)

# install lib,node and dictionary
install(TARGETS
  ${target}
  LIBRARY DESTINATION lib
)
