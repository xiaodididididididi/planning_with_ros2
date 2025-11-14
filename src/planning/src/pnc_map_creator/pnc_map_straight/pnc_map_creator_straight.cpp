#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight()
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator_straight created");
        pnc_map_config_ = std::make_unique<ConfigReader>();
        pnc_map_config_->read_pnc_map_config();
        map_type_ = static_cast<int>(PNCMapType::STRAIGHT);

        p_mid_.x = -3.0;
        p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

        len_step_ = pnc_map_config_->pnc_map().segment_len_;

        init_pnc_map();
    }

    PNCMap PNCMapCreatorStraight::creat_pnc_map()
    {
        draw_straight_x(pnc_map_.road_length, 1.0);

        if (pnc_map_.midline.points.size() % 2 == 1)
        {
            pnc_map_.midline.points.pop_back();
        }

        pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);

        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map created, midline points: %ld", pnc_map_.midline.points.size());
        return pnc_map_;
    }

    void PNCMapCreatorStraight::init_pnc_map()
    {
        pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
        pnc_map_.header.stamp = rclcpp::Clock().now();
        pnc_map_.road_length = pnc_map_config_->pnc_map().road_length_;
        pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

        pnc_map_.midline.header = pnc_map_.header;
        pnc_map_.midline.ns = "pnc_map";
        pnc_map_.midline.id = 0;
        pnc_map_.midline.action = Marker::ADD;
        pnc_map_.midline.type = Marker::LINE_LIST;
        pnc_map_.midline.scale.x = 0.05;
        pnc_map_.midline.color.a = 1.0;
        pnc_map_.midline.color.r = 0.7;
        pnc_map_.midline.color.g = 0.7;
        pnc_map_.midline.color.b = 0.0;
        pnc_map_.midline.lifetime = rclcpp::Duration::max();
        pnc_map_.midline.frame_locked = true;

        pnc_map_.left_boundary = pnc_map_.midline;
        pnc_map_.left_boundary.id = 1;
        pnc_map_.left_boundary.type = Marker::LINE_STRIP;
        pnc_map_.left_boundary.color.r = 1.0;
        pnc_map_.left_boundary.color.g = 1.0;
        pnc_map_.left_boundary.color.b = 1.0;

        pnc_map_.right_boundary = pnc_map_.left_boundary;
        pnc_map_.right_boundary.id = 2;
    }

    void PNCMapCreatorStraight::draw_straight_x(const double &length, const double &plus_flag, const double &ratio)
    {
        double len_temp = 0.0;
        while (len_temp < length)
        {
            pl_.x = p_mid_.x;
            pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
            pr_.x = p_mid_.x;
            pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

            pnc_map_.midline.points.emplace_back(p_mid_);
            pnc_map_.left_boundary.points.emplace_back(pl_);
            pnc_map_.right_boundary.points.emplace_back(pr_);

            len_temp += len_step_ * ratio;
            p_mid_.x += len_step_ * plus_flag * ratio;
        }
    }
}
