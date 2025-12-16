#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process_main")
    {
        RCLCPP_INFO(this->get_logger(), "planning_process created");
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;

        car_ = std::make_shared<MainCar>();
        for (int i = 0; i < 3; i++)
        {
            auto obs_car_ = std::make_shared<ObsCar>(i + 1);
            obses_spawn_.emplace_back(obs_car_);
        }

        tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

        buffer_ = std::make_unique<Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");

        refer_line_creator_ = std::make_shared<ReferenceLineCreator>();
        refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);

        decider_ = std::make_shared<DecisionCenter>();

        local_path_planner_ = std::make_shared<LocalPathPlanner>();
        local_speeds_planner_ = std::make_shared<LocalSpeedsPlanner>();
        local_path_pub_ = this->create_publisher<Path>("local_path", 10);

        local_trajectory_combiner_ = std::make_shared<LocalTrajectoryCombiner>();
        local_trajectory_pub_ = this->create_publisher<LocalTrajectory>("local_trajectory", 10);
    }

    bool PlanningProcess::process()
    {
        rclcpp::Rate rate(1.0);
        rate.sleep();

        if (!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(), "planning init failed!");
            return false;
        }

        timer_ = this->create_wall_timer(0.1s, std::bind(&PlanningProcess::planning_callback, this));

        return true;
    }

    bool PlanningProcess::planning_init()
    {
        vehicle_spawn(car_);
        for (const auto &obs : obses_spawn_)
        {
            vehicle_spawn(obs);
        }

        if (!connet_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "map server connect failed!");
            return false;
        }

        if (!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "map request and response failed!");
            return false;
        }

        if (!connet_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "global_path server connect failed!");
            return false;
        }

        if (!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "global_path request and response failed!");
            return false;
        }
        return true;
    }

    void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle)
    {
        TransformStamped spawn;
        spawn.header.stamp = this->now();
        spawn.header.frame_id = process_config_->pnc_map().frame_;
        spawn.child_frame_id = vehicle->child_frame();

        spawn.transform.translation.x = vehicle->loc_point().pose.position.x;
        spawn.transform.translation.y = vehicle->loc_point().pose.position.y;
        spawn.transform.translation.z = vehicle->loc_point().pose.position.z;
        spawn.transform.rotation.x = vehicle->loc_point().pose.orientation.x;
        spawn.transform.rotation.y = vehicle->loc_point().pose.orientation.y;
        spawn.transform.rotation.z = vehicle->loc_point().pose.orientation.z;
        spawn.transform.rotation.w = vehicle->loc_point().pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "vehicle %s spawned, x= %.2f, y= %.2f",
                    spawn.child_frame_id.c_str(), vehicle->loc_point().pose.position.x, vehicle->loc_point().pose.position.y);
        tf_broadcaster_->sendTransform(spawn);
    }

    void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle)
    {
        try
        {
            PoseStamped point;
            auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_, vehicle->child_frame(), tf2::TimePointZero);
            point.header = ts.header;
            point.pose.position.x = ts.transform.translation.x;
            point.pose.position.y = ts.transform.translation.y;
            point.pose.position.z = ts.transform.translation.z;
            point.pose.orientation.x = ts.transform.rotation.x;
            point.pose.orientation.y = ts.transform.rotation.y;
            point.pose.orientation.z = ts.transform.rotation.z;
            point.pose.orientation.w = ts.transform.rotation.w;
            vehicle->update_location(point);
        }
        catch (const tf2::LookupException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Lookup exception: %s", e.what());
        }
    }

    template <typename T>
    bool PlanningProcess::connet_server(const T &client)
    {
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
        {
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wrong client type!");
            return false;
        }

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
        }

        return true;
    }

    bool PlanningProcess::map_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request/");
        auto request = std::make_shared<PNCMapService::Request>();
        request->map_type = process_config_->pnc_map().type_;
        auto result_future = map_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map response success");
            pnc_map_ = result_future.get()->pnc_map;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Map response failed!");
            return false;
        }

        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending global_path request/");
        auto request = std::make_shared<GlobalPathService::Request>();
        request->pnc_map = pnc_map_;
        request->global_planner_type = process_config_->global_path().type_;
        auto result_future = global_path_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "global_path response success");
            global_path_ = result_future.get()->global_path;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "global_path response failed!");
            return false;
        }

        return false;
    }
    void PlanningProcess::planning_callback()
    {
        const auto start_time = this->get_clock()->now();

        get_location(car_);
        obses_.clear();
        for (const auto &obs : obses_spawn_)
        {
            get_location(obs);
            if (std::hypot(car_->loc_point().pose.position.x - obs->loc_point().pose.position.x,
                           car_->loc_point().pose.position.y - obs->loc_point().pose.position.y) > obs_dis_)
            {
                continue;
            }
            obses_.emplace_back(obs);
        }

        const auto refer_line = refer_line_creator_->create_reference_line(global_path_, car_->loc_point());
        if (refer_line.refer_line.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "reference line empty!");
            return;
        }
        const auto refer_line_rviz = refer_line_creator_->referline_to_rviz();
        refer_line_pub_->publish(refer_line_rviz);

        car_->vehicle_cartesian_to_frenet(refer_line);
        for (const auto &obs : obses_)
        {
            obs->vehicle_cartesian_to_frenet(refer_line);
        }

        std::sort(obses_.begin(), obses_.end(),
                  [](const std::shared_ptr<VehicleBase> &obs1, const std::shared_ptr<VehicleBase> &obs2)
                  { return obs1->s() < obs2->s(); });

        decider_->make_path_decision(car_, obses_);

        const auto local_path = local_path_planner_->create_local_path(refer_line, car_, decider_);
        if (local_path.local_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local path empty!");
            return;
        }
        const auto local_path_rviz = local_path_planner_->path_to_rviz();
        local_path_pub_->publish(local_path_rviz);

        for (const auto &obs : obses_)
        {
            obs->vehicle_cartesian_to_frenet_2path(local_path, refer_line, car_);
        }

        decider_->make_speed_decision(car_, obses_);

        const auto local_speeds = local_speeds_planner_->cal_speed(decider_);
        if (local_speeds.local_speeds.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local speeds empty!");
            return;
        }

        const auto local_trajectory = local_trajectory_combiner_->combin_trajectory(local_path, local_speeds);
        if (local_trajectory.local_trajectory.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local trajectory empty!");
            return;
        }
        local_trajectory_pub_->publish(local_trajectory);

        car_->update_cartesian_info(local_trajectory.local_trajectory[0]);

        RCLCPP_INFO(this->get_logger(), "-------car state: loc: (%.2f,%.2f),speed: %.2f, a:%.2f, theta:%.2f, kappa:%.2f",
                    car_->loc_point().pose.position.x,
                    car_->loc_point().pose.position.y,
                    car_->speed(), car_->acceleration(),
                    car_->theta(), car_->kappa());

        const auto end_time = this->get_clock()->now();
        const double planning_total_time = end_time.seconds() - start_time.seconds();
        RCLCPP_INFO(this->get_logger(), "planning total time: %fms\n", planning_total_time * 1000);

        if (planning_total_time > 1.0)
        {
            RCLCPP_ERROR(this->get_logger(), "planning time too long!");
            rclcpp::shutdown();
        }
    }
}