#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    enum class GlobalPlannerType
    {
        NORMAL
    };

    class GlobalPlannerBase
    {
    public:
        virtual Path search_global_path(const PNCMap &pnc_map) = 0;
        inline Path global_path() const { return global_path_; }
        virtual ~GlobalPlannerBase() {}

    protected:
        std::unique_ptr<ConfigReader> global_planner_config_;
        int global_planner_type_ = 0;
        Path global_path_;
    };
}

#endif