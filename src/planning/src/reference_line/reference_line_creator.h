#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
    using base_msgs::msg::Referline;
    using base_msgs::msg::ReferlinePoint;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class ReferenceLineCreator
    {
    public:
        ReferenceLineCreator();
        Referline create_reference_line(const Path &global_path, const PoseStamped &target_point);
        Path referline_to_rviz();
        inline Referline refer_line() const { return refer_line_; }
        inline Path refer_line_rviz() const { return refer_line_rviz_; }
        inline int match_point_index() const { return match_point_index_; }
        inline int front_index() const { return front_index_; }
        inline int back_index() const { return back_index_; }

    private:
        std::unique_ptr<ConfigReader> reference_line_config_;
        Referline refer_line_;
        Path refer_line_rviz_;
        std::shared_ptr<ReferenceLineSmoother> refer_line_smoother_;
        int last_match_point_index_ = -1;
        int match_point_index_ = -1;
        int front_index_ = -1;
        int back_index_ = -1;
    };
}

#endif