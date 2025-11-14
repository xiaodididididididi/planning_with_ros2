#ifndef OSQP_TEST_H
#define OSQP_TEST_h

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace Planning
{
    class OsqpTest : public rclcpp::Node
    {
    public:
        OsqpTest();
        void test_problem();
    };
}

#endif