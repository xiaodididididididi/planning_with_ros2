#include "local_path_smoother.h"

namespace Planning
{
    LocalPathSmoother::LocalPathSmoother()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_smoother created");

        local_path_config_ = std::make_unique<ConfigReader>();
        local_path_config_->read_local_path_config();
    }

    void LocalPathSmoother::smooth_local_path(LocalPath &path)
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_ smoothed");
        (void)path;
    }
}