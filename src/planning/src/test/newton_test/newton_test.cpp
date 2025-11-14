#include "newton_test.h"

namespace Planning
{
    NewtonTest::NewtonTest() : Node("newton_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "newton_test_node created");
        newton_problem();
    }

    double NewtonTest::function(double x)
    {
        return x * x * x * x + 2 * x;
    }

    double NewtonTest::dfunction(double x)
    {
        return 4 * x * x * x + 2;
    }

    double NewtonTest::ddfunction(double x)
    {
        return 12 * x * x;
    }

    void NewtonTest::newton_problem()
    {
        double x = 100.0;
        while (abs(dfunction(x)) > 1e-6)
        {
            x = x - dfunction(x) / (ddfunction(x) + 1e-6);
        }
        std::cout << "x is:" << x << std::endl;
        std::cout << "min value is:" << function(x) << std::endl;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::NewtonTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}