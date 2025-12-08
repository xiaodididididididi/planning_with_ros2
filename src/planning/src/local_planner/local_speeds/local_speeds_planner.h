#ifndef LOCAL_SPEEDS_PLANNER_H_
#define LOCAL_SPEEDS_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "config_reader.h"
#include "polynomial_curve.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"

namespace Planning
{

    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalSpeedsPoint;
    class LocalSpeedsPlanner
    {
    public:
        LocalSpeedsPlanner();
        LocalSpeeds cal_speed(const std::shared_ptr<DecisionCenter> &decision);
        inline LocalSpeeds local_speeds() const { return local_speeds_; }

    private:
        void init_local_speeds();

        std::unique_ptr<ConfigReader> local_speeds_config_;
        LocalSpeeds local_speeds_;
        std::shared_ptr<LocalSpeedsSmoother> local_speeds_smoother_;
    };
}

#endif