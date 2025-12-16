#include "local_speeds_smoother.h"

namespace Planning
{
    LocalSpeedsSmoother::LocalSpeedsSmoother()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "local_speeds_smoother created");
        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_speeds_config();
    }

    void LocalSpeedsSmoother::smooth_local_sppeds(LocalSpeeds speeds)
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "local speeds smoothed");
        (void)speeds;
    }
}