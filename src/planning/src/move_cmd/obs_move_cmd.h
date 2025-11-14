#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "config_reader.h"
#include "obs_car_info.h"

namespace Planning
{
    using namespace std::chrono_literals;
    using geometry_msgs::msg::TransformStamped;
    using tf2_ros::TransformBroadcaster;

    struct ObsParam
    {
        std::shared_ptr<VehicleBase> obs_;
        std::shared_ptr<TransformBroadcaster> obs_broadcaster_;
        double pos_x_ = 0.0;
        double pos_y_ = 0.0;
        double theta_ = 0.0;
        double speed_ = 0.0;
    };

    class ObsMoveCmd : public rclcpp::Node
    {
    public:
        ObsMoveCmd();

    private:
        void obs_broadcast_tf();
        std::unique_ptr<ConfigReader> obs_move_cmd_config_;
        std::vector<ObsParam> obses_param_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif