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
        init_local_speeds();

        double point_t = 0.0;
        LocalSpeedsPoint point_tmp;
        for (int i = 0; i < local_speeds_config_->local_speeds().speed_size_; i++)
        {
            point_t += 1.0;
            if (point_t > local_speeds_config_->local_speeds().speed_size_)
            {
                break;
            }

            point_tmp.t = point_t;
            point_tmp.s_2path = local_speeds_config_->main_car().speed_ori_ * point_t;
            point_tmp.ds_dt_2path = local_speeds_config_->main_car().speed_ori_;
            point_tmp.dds_dt_2path = 0.0;
            const int st_points_size = decision->st_points().size();

            for (int j = 0; j < st_points_size - 1; j++)
            {
                const double start_t = decision->st_points()[j].t_;
                const double start_s = decision->st_points()[j].s_2path_;
                const double start_ds_dt = decision->st_points()[j].ds_dt_2path_;
                const double start_dds_dt = 0.0;

                const double end_t = decision->st_points()[j + 1].t_;
                const double end_s = decision->st_points()[j + 1].s_2path_;
                const double end_ds_dt = decision->st_points()[j + 1].ds_dt_2path_;
                const double end_dds_dt = 0.0;

                if (point_t >= start_t && point_t <= end_t)
                {
                    if (end_t == decision->st_points().back().t_ && end_s == decision->st_points().back().s_2path_)
                    {
                        const Eigen::Vector2d a = PolynomialCurve::linear_polynomial(start_t, start_s, end_t, end_s);
                        point_tmp.s_2path = a(0) + a(1) * point_t;
                        point_tmp.ds_dt_2path = a(1);
                        point_tmp.dds_dt_2path = 0.0;

                        if (fabs(point_tmp.ds_dt_2path) < min_speed)
                        {
                            point_tmp.ds_dt_2path = 0.0;
                        }
                    }
                    else
                    {
                        const double point_t_2 = point_t * point_t;
                        const double point_t_3 = point_t_2 * point_t;
                        const double point_t_4 = point_t_3 * point_t;
                        const double point_t_5 = point_t_4 * point_t;

                        const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(start_t, start_s, start_ds_dt, start_dds_dt, end_t, end_s, end_ds_dt, end_dds_dt);
                        point_tmp.s_2path = a(0) + a(1) * point_t + a(2) * point_t_2 + a(3) * point_t_3 + a(4) * point_t_4 + a(5) * point_t_5;
                        point_tmp.ds_dt_2path = a(1) + 2 * a(2) * point_t + 3 * a(3) * point_t_2 + 4 * a(4) * point_t_3 + 5 * a(5) * point_t_4;
                        point_tmp.dds_dt_2path = 2 * a(2) + 6 * a(3) * point_t + 12 * a(4) * point_t_2 + 20 * a(5) * point_t_3;
                    }
                }
            }

            point_tmp.speed = point_tmp.ds_dt_2path;
            point_tmp.acceleration = point_tmp.dds_dt_2path;
            local_speeds_.local_speeds.emplace_back(point_tmp);
        }

        local_speeds_smoother_->smooth_local_sppeds(local_speeds_);
        RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "local speeds created,size=%ld", local_speeds_.local_speeds.size());

        return local_speeds_;
    }

    void LocalSpeedsPlanner::init_local_speeds()
    {
        local_speeds_.header.frame_id = local_speeds_config_->pnc_map().frame_;
        local_speeds_.header.stamp = rclcpp::Clock().now();
        local_speeds_.local_speeds.clear();
    }
}