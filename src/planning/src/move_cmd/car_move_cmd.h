#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "config_reader.h"
#include "main_car_info.h"

namespace Planning
{
    using namespace std::chrono_literals;
    using base_msgs::msg::LocalTrajectory;
    using geometry_msgs::msg::TransformStamped;
    using std::placeholders::_1;
    using tf2_ros::TransformBroadcaster;

    struct car_param
    {
        double pos_x_ = 0.0;
        double pos_y_ = 0.0;
        double theta_ = 0.0;
        double speed_ = 0.0;
    };

    class CarMoveCmd : public rclcpp::Node
    {
    public:
        CarMoveCmd();

    private:
        std::unique_ptr<ConfigReader> move_cmd_config_;
        std::shared_ptr<TransformBroadcaster> broadcaster_;
        rclcpp::Subscription<LocalTrajectory>::SharedPtr local_trajectory_sub_;
        std::shared_ptr<VehicleBase> car_;
        car_param car_param_;

        void car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory);
    };
}

#endif