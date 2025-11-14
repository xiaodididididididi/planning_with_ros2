#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include <vector>

namespace Planning
{
    constexpr double min_speed = 0.03;

    enum class SLPointType
    {
        LFET_PASS,
        RIGHT_PASS,
        STOP,
        START,
        END
    };

    enum class STPointType
    {
        GIVE_WAY,
        RUSH_OUT,
        STOP,
        START,
        END
    };

    struct SLPoint
    {
        double s_ = 0.0;
        double l_ = 0.0;
        int type_ = 0;
    };

    struct STPoint
    {
        double t_ = 0.0;
        double s_2path_ = 0.0;
        double ds_dt_2path_ = 0;
        double t0_ = 0.0;
        double s0_ = 0.0;
        int type_ = 0;
    };

    class DecisionCenter
    {
    public:
        DecisionCenter();
        void make_path_decision(const std::shared_ptr<VehicleBase> &car,
                                const std::vector<std::shared_ptr<VehicleBase>> &obses);
        void make_speed_decision(const std::shared_ptr<VehicleBase> &car,
                                 const std::vector<std::shared_ptr<VehicleBase>> &obses);
        inline std::vector<SLPoint> sl_points() const { return sl_points_; }
        inline std::vector<STPoint> st_points() const { return st_points_; }

    private:
        std::unique_ptr<ConfigReader> decision_config_;
        std::vector<SLPoint> sl_points_;
        std::vector<STPoint> st_points_;
    };
}

#endif