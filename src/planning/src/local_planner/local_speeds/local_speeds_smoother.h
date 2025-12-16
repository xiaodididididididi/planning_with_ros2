#ifndef LOCAL_SPEEDS_SMOOTHER_H_
#define LOCAL_SPEEDS_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/local_speeds.hpp"
#include <cmath>

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;
    class LocalSpeedsSmoother
    {
    public:
        LocalSpeedsSmoother();
        void smooth_local_sppeds(LocalSpeeds speeds);

    private:
        std::unique_ptr<ConfigReader> local_speeds_config_;
    };
}

#endif