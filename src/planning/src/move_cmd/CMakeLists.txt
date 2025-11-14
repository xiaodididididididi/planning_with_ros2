cmake_minimum_required(VERSION 3.31.3)
project(move_cmd)

# set depends
set(DEPENDS
  "rclcpp"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "base_msgs"
)

# add node
add_executable(car_move_cmd
  ${CMAKE_CURRENT_SOURCE_DIR}/car_move_cmd.cpp
)

add_executable(obs_move_cmd
  ${CMAKE_CURRENT_SOURCE_DIR}/obs_move_cmd.cpp
)

set(MOVE_TARGETS
  car_move_cmd
  obs_move_cmd
)

# add depends
foreach(target ${MOVE_TARGETS})
  ament_target_dependencies(${target}
    ${DEPENDS}
  )

  target_link_libraries(${target}
    config_reader
    vehicle_info
  )
endforeach()

# switch
option(USE_ACTUAL_POS_MACRO "Enable USE_ACTUAL_POS" OFF)

if(USE_ACTUAL_POS_MACRO)
  target_compile_definitions(car_move_cmd PUBLIC USE_ACTUAL_POS)
endif()

# install lib,node and dictionary
install(TARGETS
  ${MOVE_TARGETS}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)
