#include "local_trajectory_combiner.h"

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "local_trajectory_combiner created");

        trajectory_config_ = std::make_unique<ConfigReader>();
    }

    LocalTrajectory LocalTrajectoryCombiner::combin_trajectory(const LocalPath &path, const LocalSpeeds &speeds)
    {
        const int path_size = path.local_path.size();
        const int speeds_size = speeds.local_speeds.size();
        local_trajector_.header = path.header;
        local_trajector_.local_trajectory.clear();

        if (path_size < 3)
        {
            RCLCPP_WARN(rclcpp::get_logger("tragectory"), "local path or speed empty!");
            return local_trajector_;
        }

        LocalTrajectoryPoint point_tmp;
        for (int i = 0; i < path_size; i++)
        {
            point_tmp.path_point = path.local_path[i];

            local_trajector_.local_trajectory.emplace_back(point_tmp);
        }

        RCLCPP_INFO(rclcpp::get_logger("tragectory"), "tragectory combined, size = %ld", local_trajector_.local_trajectory.size());
        return local_trajector_;
    }
}