#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
  using namespace std::chrono_literals;
  using base_msgs::msg::LocalTrajectory;
  using base_msgs::msg::PNCMap;
  using base_msgs::srv::GlobalPathService;
  using base_msgs::srv::PNCMapService;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;
  using tf2_ros::Buffer;
  using tf2_ros::StaticTransformBroadcaster;
  using tf2_ros::TransformListener;

  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess();
    bool process();
    inline PNCMap pnc_map() const { return pnc_map_; }
    inline Path global_path() const { return global_path_; }

  private:
    std::unique_ptr<ConfigReader> process_config_;
    std::shared_ptr<VehicleBase> car_;
    std::vector<std::shared_ptr<VehicleBase>> obses_spawn_;
    std::vector<std::shared_ptr<VehicleBase>> obses_;
    double obs_dis_ = 0.0;

    std::shared_ptr<StaticTransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<Buffer> buffer_;
    std::shared_ptr<TransformListener> tf_listener_;

    PNCMap pnc_map_;
    Path global_path_;
    rclcpp::Client<PNCMapService>::SharedPtr map_client_;
    rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_;

    std::shared_ptr<ReferenceLineCreator> refer_line_creator_;
    rclcpp::Publisher<Path>::SharedPtr refer_line_pub_;

    std::shared_ptr<DecisionCenter> decider_;

    std::shared_ptr<LocalPathPlanner> local_path_planner_;
    std::shared_ptr<LocalSpeedsPlanner> local_speeds_planner_;
    rclcpp::Publisher<Path>::SharedPtr local_path_pub_;

    std::shared_ptr<LocalTrajectoryCombiner> local_trajectory_combiner_;
    rclcpp::Publisher<LocalTrajectory>::SharedPtr local_trajectory_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool planning_init();

    void vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle);
    void get_location(const std::shared_ptr<VehicleBase> &vehicle);

    template <typename T>
    bool connet_server(const T &client);
    bool map_request();
    bool global_path_request();
    void planning_callback();
  };
}

#endif