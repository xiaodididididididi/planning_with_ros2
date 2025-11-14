#include "obs_car_info.h"

namespace Planning
{
    ObsCar::ObsCar(const int &id)
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car created");

        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config();

        child_frame_ = vehicle_config_->obs_pair()[id].frame_;
        length_ = vehicle_config_->obs_pair()[id].length_;
        width_ = vehicle_config_->obs_pair()[id].width_;
        theta_ = vehicle_config_->obs_pair()[id].pose_theta_;
        speed_ = vehicle_config_->obs_pair()[id].speed_ori_;
        id_ = id;

        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, theta_);
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->obs_pair()[id].pose_x_;
        loc_point_.pose.position.y = vehicle_config_->obs_pair()[id].pose_y_;
        loc_point_.pose.position.z = 0.0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }

    void ObsCar::vehicle_cartesian_to_frenet(const Referline &refer_line)
    {
        double rs, rx, ry, rtheta, rkappa, rdkappa;

        Curve::find_projection_point(refer_line, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car projection_point: rs=%.2f, rx=%.2f, ry=%.2f, rtheta=%.2f, rkappa=%.2f, rdkappa=%.2f",
                    rs, rx, ry, rtheta, rkappa, rdkappa);

        Curve::cartesian_to_frenet(loc_point_.pose.position.x,
                                   loc_point_.pose.position.y,
                                   theta_, speed_, acceleration_, kappa_,
                                   rs, rx, ry, rtheta, rkappa, rdkappa,
                                   s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car cartesian_to_frenet: s=%.2f, ds_dt=%.2f, dds_dt=%.2f, l=%.2f, dl_ds=%.2f, ddl_ds=%.2f, dl_dt=%.2f, ddl_dt=%.2f",
                    s_, ds_dt_, dds_dt_, l_, dl_ds_, ddl_ds_, dl_dt_, ddl_dt_);
    }

    void ObsCar::vehicle_cartesian_to_frenet_2path(const LocalPath &local_path, const Referline &refer_line, const std::shared_ptr<VehicleBase> &car)
    {
        const double path0_index = Curve::find_match_point(refer_line, local_path.local_path[0].pose);
        const double path_end_index = Curve::find_match_point(refer_line, local_path.local_path.back().pose);

        if (s_ > refer_line.refer_line[path_end_index].rs || s_ < refer_line.refer_line[path0_index].rs)
        {
            s_2path_ = s_ - refer_line.refer_line[path0_index].rs;
            ds_dt_2path_ = ds_dt_;
            l_2path_ = l_ - car->l();
            dl_ds_2path_ = dl_ds_;
            dl_dt_2path_ = dl_dt_;
            dds_dt_2path_ = dds_dt_;
            ddl_ds_2path_ = ddl_ds_;
            ddl_dt_2path_ = ddl_dt_;

            RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car cartesian to frenet to path: path0_rs=%.2f,path_end_rs=%.2f,s2path=%.2f,ds_dt_2path=%.2f,l2path=%.2f,dl_ds_2path=%.2f,dl_dt_2path=%.2f",
                        refer_line.refer_line[path0_index].rs, refer_line.refer_line[path_end_index].rs, s_2path_, ds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_);
            return;
        }

        double rs, rx, ry, rtheta, rkappa, rdkappa;

        Curve::find_projection_point(local_path, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car projection point to path: rs=%.2f,rx=%.2f,ry=%.2f,rtheta=%.2f,rkappa=%.2f,rdkappa=%.2f",
                    rs, rx, ry, rtheta, rkappa, rdkappa);

        Curve::cartesian_to_frenet(loc_point_.pose.position.x,
                                   loc_point_.pose.position.y,
                                   theta_, speed_, acceleration_, kappa_,
                                   rs, rx, ry, rtheta, rkappa, rdkappa,
                                   s_2path_, ds_dt_2path_, dds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_, ddl_ds_2path_, ddl_dt_2path_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car cartesian to frenet to path: s2path=%.2f,ds_dt_2path=%.2f,l2path=%.2f,dl_ds_2path=%.2f,dl_dt_2path=%.2f",
                    s_2path_, ds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_);
    }
}