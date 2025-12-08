#include "local_speeds_planner.h"

namespace Planning
{
    LocalSpeedsPlanner::LocalSpeedsPlanner()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "local_speeds_planner created");

        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_speeds_config();
        local_speeds_smoother_ = std::make_shared<LocalSpeedsSmoother>();
    }

    LocalSpeeds LocalSpeedsPlanner::cal_speed(const std::shared_ptr<DecisionCenter> &decision)
    {
        return local_speeds_;
    }

    void LocalSpeedsPlanner::init_local_speeds()
    {
        local_speeds_.header.frame_id = local_speeds_config_->pnc_map().frame_;
        local_speeds_.header.stamp = rclcpp::Clock().now();
        local_speeds_.local_speeds.clear();
    }
}