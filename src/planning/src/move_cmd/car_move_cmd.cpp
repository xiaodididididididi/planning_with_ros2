#include "car_move_cmd.h"

namespace Planning
{

    MPCSolver::MPCSolver(const MPCConfig &conf) : c(conf)
    {
        vars_n = (c.N + 1) * c.nx + c.N * c.nu;
        cons_n = (c.N + 1) * c.nx + c.N * c.nu + (c.N - 1);
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
    }

    Eigen::Vector2d MPCSolver::solve(const Eigen::Vector3d &x0, const std::vector<Eigen::Vector3d> &xref, double last_v)
    {
        Eigen::DiagonalMatrix<double, 3> Q;
        Q.diagonal() << 100, 100, 50;
        Eigen::DiagonalMatrix<double, 2> R;
        R.diagonal() << 1, 1;

        Eigen::SparseMatrix<double> P(vars_n, vars_n);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(vars_n);
        std::vector<Eigen::Triplet<double>> p_triplets;

        for (int i = 0; i < c.N; ++i)
        {
            for (int j = 0; j < c.nx; ++j)
            {
                p_triplets.push_back({i * c.nx + j, i * c.nx + j, Q.diagonal()[j]});
            }
            if (i < c.N)
                q.segment(i * c.nx, c.nx) = -1 * Q * (i < (int)xref.size() ? xref[i] : xref.back());
        }
        int u_off = (c.N + 1) * c.nx;
        for (int i = 0; i < c.N; ++i)
        {
            for (int j = 0; j < c.nu; ++j)
            {
                p_triplets.push_back({u_off + i * c.nu + j, u_off + i * c.nu + j, R.diagonal()[j]});
            }
        }
        P.setFromTriplets(p_triplets.begin(), p_triplets.end());

        Eigen::SparseMatrix<double> A(cons_n, vars_n);
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(cons_n, -1e6);
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(cons_n, 1e6);
        std::vector<Eigen::Triplet<double>> a_triplets;

        for (int j = 0; j < c.nx; ++j)
        {
            a_triplets.push_back({j, j, 1.0});
            lb(j) = ub(j) = x0(j);
        }

        for (int i = 0; i < c.N; ++i)
        {
            Eigen::Vector3d ref = (i < (int)xref.size() ? xref[i] : xref.back());
            double psi_r = ref(2);
            double v_r = std::max(0.3, last_v);

            Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();
            Ad(0, 2) = -v_r * sin(psi_r) * c.dt;
            Ad(1, 2) = v_r * cos(psi_r) * c.dt;

            Eigen::Matrix<double, 3, 2> Bd;
            Bd << cos(psi_r) * c.dt, 0, sin(psi_r) * c.dt, 0, 0, v_r * c.dt / c.L;

            int row = (i + 1) * c.nx;
            for (int r = 0; r < 3; r++)
            {
                a_triplets.push_back({row + r, row + r, 1.0});
                for (int col = 0; col < 3; col++)
                {
                    a_triplets.push_back({row + r, i * c.nx + col, -Ad(r, col)});
                }
                for (int col = 0; col < 2; col++)
                {
                    a_triplets.push_back({row + r, u_off + i * c.nu + col, -Bd(r, col)});
                }
            }

            lb.segment(row, 3).setZero();
            ub.segment(row, 3).setZero();
        }

        int cons_idx = (c.N + 1) * c.nx;
        for (int i = 0; i < c.N; ++i)
        {
            int u_idx = u_off + i * c.nu;
            a_triplets.push_back({cons_idx, u_idx, 1.0});
            lb(cons_idx) = c.v_min;
            ub(cons_idx) = c.v_max;
            a_triplets.push_back({cons_idx + 1, u_idx + 1, 1.0});
            lb(cons_idx + 1) = c.delta_min;
            ub(cons_idx + 1) = c.delta_max;

            if (i > 0)
            {
                int acc_row = (c.N + 1) * c.nx + c.N * c.nu + (i - 1);
                a_triplets.push_back({acc_row, u_idx, 1.0});
                a_triplets.push_back({acc_row, u_idx - c.nu, -1.0});
                lb(acc_row) = c.acc_min * c.dt;
                ub(acc_row) = c.acc_max * c.dt;
            }
            else
            {
                lb(cons_idx) = std::max(c.v_min, last_v + c.acc_min * c.dt);
                ub(cons_idx) = std::min(c.v_max, last_v + c.acc_max * c.dt);
            }
            cons_idx += 2;
        }
        A.setFromTriplets(a_triplets.begin(), a_triplets.end());

        solver.clearSolver();
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
        solver.data()->setNumberOfVariables(vars_n);
        solver.data()->setNumberOfConstraints(cons_n);
        if (!solver.data()->setHessianMatrix(P))
            return {0, 0};
        solver.data()->setGradient(q);
        if (!solver.data()->setLinearConstraintsMatrix(A))
            return {0, 0};
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);

        if (solver.initSolver())
        {
            solver.solveProblem();
            return solver.getSolution().segment(u_off, 2);
        }
        else
        {
            std::cout << "MPC Solver init failed!" << std::endl;
        }
        return {0, 0};
    }

    CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node")
    {
        RCLCPP_INFO(this->get_logger(), "car_move_cmd_node created");

        move_cmd_config_ = std::make_unique<ConfigReader>();
        move_cmd_config_->read_move_cmd_config();

        car_ = std::make_shared<MainCar>();
        car_param_.pos_x_ = car_->loc_point().pose.position.x;
        car_param_.pos_y_ = car_->loc_point().pose.position.y;
        car_param_.theta_ = car_->theta();
        car_param_.speed_ = car_->speed();

        mpc_ptr_ = std::make_unique<MPCSolver>(mpc_cfg_);

        broadcaster_ = std::make_shared<TransformBroadcaster>(this);

        local_trajectory_sub_ = this->create_subscription<LocalTrajectory>(
            "planning/local_trajectory",
            10,
            std::bind(&CarMoveCmd::car_broadcast_tf, this, _1));
    }

    void CarMoveCmd::car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory)
    {
        const int trajectory_size = trajectory->local_trajectory.size();
        if (trajectory_size < 3)
        {
            RCLCPP_WARN(this->get_logger(), "local_trajectory empty!");
            return;
        }

        TransformStamped transform_data;
        transform_data.header.stamp = trajectory->header.stamp;
        transform_data.header.frame_id = move_cmd_config_->pnc_map().frame_;
        transform_data.child_frame_id = car_->child_frame();

        double min_dis = std::numeric_limits<double>::max();
        int closest_index = -1;
        for (int i = 0; i < trajectory_size; i++)
        {
            double dis = std::hypot(trajectory->local_trajectory[i].path_point.pose.pose.position.x - car_param_.pos_x_,
                                    trajectory->local_trajectory[i].path_point.pose.pose.position.y - car_param_.pos_y_);
            if (dis < min_dis)
            {
                min_dis = dis;
                closest_index = i;
            }
        }

        std::vector<Eigen::Vector3d> xref;
        for (int i = 0; i < mpc_cfg_.N; ++i)
        {
            int idx = std::min(closest_index + i, trajectory_size - 1);
            auto &pt = trajectory->local_trajectory[idx].path_point;
            xref.push_back({pt.pose.pose.position.x, pt.pose.pose.position.y, pt.theta});
        }

        Eigen::Vector3d current_state(car_param_.pos_x_, car_param_.pos_y_, car_param_.theta_);
        Eigen::Vector2d control = mpc_ptr_->solve(current_state, xref, car_param_.speed_);

        double v = control(0);
        double delta = control(1);
        double dt = mpc_cfg_.dt;

        car_param_.pos_x_ += v * std::cos(car_param_.theta_) * dt;
        car_param_.pos_y_ += std::sin(car_param_.theta_) * dt;
        car_param_.theta_ += v * std::tan(delta) / mpc_cfg_.L * dt;
        car_param_.speed_ = v;

        // 跟踪误差
        auto &ref_pt = trajectory->local_trajectory[closest_index].path_point;
        double ref_x = ref_pt.pose.pose.position.x;
        double ref_y = ref_pt.pose.pose.position.y;
        double ref_theta = ref_pt.theta;

        double dx = car_param_.pos_x_ - ref_x;
        double dy = car_param_.pos_y_ - ref_y;

        // 计算横向误差 (Lateral Error)
        double lateral_error = -dx * std::sin(ref_theta) + dy * std::cos(ref_theta);

        // 计算航向误差 (Heading Error)
        double heading_error = car_param_.theta_ - ref_theta;
        while (heading_error > M_PI)
            heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2.0 * M_PI;
        RCLCPP_INFO(this->get_logger(),
                    "MPC Output -> v: %.2f, delta: %.2f | Error -> Lat: %.4f m, Head: %.4f rad",
                    v, delta, lateral_error, heading_error);

        // TF广播
        transform_data.transform.translation.x = car_param_.pos_x_;
        transform_data.transform.translation.y = car_param_.pos_y_;
        transform_data.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, car_param_.theta_);
        transform_data.transform.rotation.x = q.x();
        transform_data.transform.rotation.y = q.y();
        transform_data.transform.rotation.z = q.z();
        transform_data.transform.rotation.w = q.w();

        broadcaster_->sendTransform(transform_data);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::CarMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}