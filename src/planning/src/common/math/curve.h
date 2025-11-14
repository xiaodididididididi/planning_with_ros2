#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    constexpr double delta_s_min = 1.0;
    constexpr double kMathEpsilon = 1.0e-6;

    class Curve
    {
    public:
        Curve() = default;

        static double NormalizeAngle(const double &angle);

        static void cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa,
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa,
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt);

        static void frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds,
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa,
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa);

        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point);
        static int find_match_point(const Referline &path, const PoseStamped &target_point);
        static int find_match_point(const LocalPath &path, const PoseStamped &target_point);
        static int find_match_point(const Referline &path, const double &rs);

        static void find_projection_point(const Referline &referline, const PoseStamped &target_point,
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa);
        static void find_projection_point(const LocalPath &path, const PoseStamped &target_point,
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa);

        static void cal_projection_param(Referline &refer_line);
        static void cal_projection_param(LocalPath &local_path);
    };
}

#endif
