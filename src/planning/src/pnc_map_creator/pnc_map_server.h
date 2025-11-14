#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "pnc_map_creator_straight.h"
#include "pnc_map_creator_sturn.h"

namespace Planning
{
    using base_msgs::srv::PNCMapService;
    using std::placeholders::_1;
    using std::placeholders::_2;

    class PNCMapServer : public rclcpp::Node
    {
    public:
        PNCMapServer();

    private:
        std::shared_ptr<PNCMapCreatorBase> map_creator_;
        rclcpp::Publisher<PNCMap>::SharedPtr map_pub_;
        rclcpp::Publisher<MarkerArray>::SharedPtr map_rviz_pub_;
        rclcpp::Service<PNCMapService>::SharedPtr map_server_;

        void response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                       const std::shared_ptr<PNCMapService::Response> response);
    };
}

#endif