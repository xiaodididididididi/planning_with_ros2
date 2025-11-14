#include "car_move_cmd.h"

namespace Planning
{
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

        const double speed_x = 1.0 * std::cos(trajectory->local_trajectory[closest_index].path_point.theta);
        const double speed_y = 1.0 * std::sin(trajectory->local_trajectory[closest_index].path_point.theta);

#ifdef USE_ACTUAL_POS
        transform_data.transform.translation.x = car_param_.pos_x_;
        transform_data.transform.translation.y = car_param_.pos_y_;
#else
        transform_data.transform.translation.x = trajectory->local_trajectory[closest_index].path_point.pose.pose.position.x;
        transform_data.transform.translation.y = trajectory->local_trajectory[closest_index].path_point.pose.pose.position.y;
#endif

        car_param_.pos_x_ += speed_x;
        car_param_.pos_y_ += speed_y;

        transform_data.transform.rotation.x = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.x;
        transform_data.transform.rotation.y = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.y;
        transform_data.transform.rotation.z = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.z;
        transform_data.transform.rotation.w = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "move_cmd broadcasted, pos:(%.2f,%.2f),speed:(%.2f,%.2f),local_trajectory_size:%ld",
                    car_param_.pos_x_, car_param_.pos_y_, speed_x, speed_y, trajectory->local_trajectory.size());
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