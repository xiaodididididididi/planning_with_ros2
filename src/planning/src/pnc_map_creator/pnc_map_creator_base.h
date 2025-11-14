#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType
    {
        STRAIGHT,
        STURN
    };

    class PNCMapCreatorBase
    {
    public:
        virtual PNCMap creat_pnc_map() = 0;
        inline PNCMap pnc_map() const { return pnc_map_; }
        inline MarkerArray pnc_map_markerarray() const { return pnc_map_markerarray_; }
        virtual ~PNCMapCreatorBase() {}

    protected:
        std::unique_ptr<ConfigReader> pnc_map_config_;
        int map_type_ = 0;
        PNCMap pnc_map_;
        MarkerArray pnc_map_markerarray_;

        Point p_mid_, pl_, pr_;
        double theta_current_ = 0.0;
        double len_step_ = 0.0;
        double theta_step_ = 0.0;
    };
}

#endif