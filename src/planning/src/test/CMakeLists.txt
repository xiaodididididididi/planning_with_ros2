cmake_minimum_required(VERSION 3.31.3)
project(test)

# set depends
set(DEPENDS
    "rclcpp"
)

# add node
add_executable(osqp_test
    ${CMAKE_CURRENT_SOURCE_DIR}/osqp_test/osqp_test.cpp
)
add_executable(newton_test
    ${CMAKE_CURRENT_SOURCE_DIR}/newton_test/newton_test.cpp
)

# add depends
ament_target_dependencies(osqp_test
    ${DEPENDS}
)
ament_target_dependencies(newton_test
    ${DEPENDS}
)

target_link_libraries(osqp_test
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)
target_link_libraries(newton_test
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

# install lib,node and dirctionary
install(TARGETS
    osqp_test
    newton_test
    DESTINATION lib/${PLANNING_PACKAGE_NAME})