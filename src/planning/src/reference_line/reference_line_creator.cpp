#include "reference_line_creator.h"

namespace Planning
{
    ReferenceLineCreator::ReferenceLineCreator()
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator created");

        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
        refer_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
    }

    Referline ReferenceLineCreator::create_reference_line(const Path &global_path, const PoseStamped &target_point)
    {
        if (global_path.poses.empty())
        {
            return refer_line_;
        }

        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_;
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "find match_point_index failed");
            return refer_line_;
        }

        const int global_path_size = global_path.poses.size();
        front_index_ = (global_path_size - 1 >= match_point_index_ + reference_line_config_->refer_line().front_size_)
                           ? (match_point_index_ + reference_line_config_->refer_line().front_size_)
                           : (global_path_size - 1);
        back_index_ = (0 <= match_point_index_ - reference_line_config_->refer_line().back_size_)
                          ? (match_point_index_ - reference_line_config_->refer_line().back_size_)
                          : 0;

        refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_;
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear();
        ReferlinePoint point_tmp;
        for (int i = back_index_; i <= front_index_; i++)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }

        refer_line_smoother_->smooth_reference_line(refer_line_);

        Curve::cal_projection_param(refer_line_);

        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference line created,match_point_index= %d,front_index= %d,back_index= %d,size= %ld",
                    match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());
        return refer_line_;
    }

    Path ReferenceLineCreator::referline_to_rviz()
    {
        refer_line_rviz_.header = refer_line_.header;
        refer_line_rviz_.poses.clear();

        PoseStamped point_tmp;
        for (const auto &point : refer_line_.refer_line)
        {
            point_tmp.header = refer_line_rviz_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }
        return refer_line_rviz_;
    }
}