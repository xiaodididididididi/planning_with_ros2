#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "config_reader.h"
#include "main_car_info.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <tf2/LinearMath/Quaternion.h>

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

    struct MPCConfig
    {
        const double L = 1.9;  // 轴距
        const double dt = 0.1; // 采样周期
        const int N = 10;      // 预测时域
        const int nx = 3;      // 状态个数
        const int nu = 2;      // 控制输入个数
        double v_min = 0.0, v_max = 5.0;
        double delta_min = -0.6, delta_max = 0.6;
        double acc_min = -2.0, acc_max = 2.0;
    };

    class MPCSolver
    {
    public:
        MPCSolver(const MPCConfig &conf);
        Eigen::Vector2d solve(const Eigen::Vector3d &x0, const std::vector<Eigen::Vector3d> &xref, double last_v);

    private:
        MPCConfig c;
        OsqpEigen::Solver solver;
        int vars_n, cons_n;
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

        std::unique_ptr<MPCSolver> mpc_ptr_;
        MPCConfig mpc_cfg_;

        void car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory);
    };
}

#endif