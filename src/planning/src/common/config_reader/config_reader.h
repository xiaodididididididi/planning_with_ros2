#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <unordered_map>

namespace Planning
{
    struct VehicleStruct
    {
        int id_ = 0;
        std::string frame_ = "";
        double length_ = 0.0;
        double width_ = 0.0;
        double pose_x_ = 0.0;
        double pose_y_ = 0.0;
        double pose_theta_ = 0.0;
        double speed_ori_ = 0.0;
    };

    struct PNCMapStruct
    {
        std::string frame_ = "";
        int type_ = 0;
        double road_length_ = 0.0;
        double road_half_width_ = 0.0;
        double segment_len_ = 0.0;
        double speed_limit_ = 0.0;
    };

    struct GlobalPathStruct
    {
        int type_ = 0;
    };

    struct ReferenceLineStruct
    {
        int type_ = 0;
        int front_size_ = 0;
        int back_size_ = 0;
    };

    struct LocalPathStruct
    {
        int curve_type_ = 0;
        int path_size_ = 0;
    };

    struct LocalSpeedsStruct
    {
        int speed_size_ = 0;
    };

    struct DecisonStruct
    {
        double safe_dis_l_ = 0.0;
        double safe_dis_s_ = 0.0;
    };

    struct ProcessStruct
    {
        double obs_dis_ = 0.0;
    };

    class ConfigReader
    {
    public:
        ConfigReader();
        void read_vehicle_config(VehicleStruct &vehicle, const std::string &name);
        void read_vehicles_config();
        inline VehicleStruct main_car() const { return main_car_; }
        inline VehicleStruct obs_car1() const { return obs_car1_; }
        inline VehicleStruct obs_car2() const { return obs_car2_; }
        inline VehicleStruct obs_car3() const { return obs_car3_; }
        inline std::unordered_map<int, VehicleStruct> obs_pair() const { return obs_pair_; }
        void read_pnc_map_config();
        inline PNCMapStruct pnc_map() const { return pnc_map_; }
        void read_global_path_config();
        inline GlobalPathStruct global_path() const { return global_path_; }
        void read_reference_line_config();
        inline ReferenceLineStruct refer_line() const { return refer_line_; }
        void read_local_path_config();
        inline LocalPathStruct local_path() const { return local_path_; }
        void read_local_speeds_config();
        inline LocalSpeedsStruct local_speeds() const { return local_speeds_; }
        void read_decision_config();
        inline DecisonStruct decision() const { return decision_; }
        void read_planning_process_config();
        inline ProcessStruct process() const { return process_; }
        void read_move_cmd_config();

    private:
        YAML::Node planning_config;
        VehicleStruct main_car_;
        VehicleStruct obs_car1_;
        VehicleStruct obs_car2_;
        VehicleStruct obs_car3_;
        std::unordered_map<int, VehicleStruct> obs_pair_;
        PNCMapStruct pnc_map_;
        GlobalPathStruct global_path_;
        ReferenceLineStruct refer_line_;
        LocalPathStruct local_path_;
        LocalSpeedsStruct local_speeds_;
        DecisonStruct decision_;
        ProcessStruct process_;
    };
}

#endif