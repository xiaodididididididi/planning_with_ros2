#ifndef NEWTON_TEST_H
#define NEWTON_TEST_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace Planning
{
    class NewtonTest : public rclcpp::Node
    {
    public:
        NewtonTest();
        double function(double x);
        double dfunction(double x);
        double ddfunction(double x);
        void newton_problem();
    };
}

#endif
