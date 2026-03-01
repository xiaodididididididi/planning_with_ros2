#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"
#include <vector>
#include <queue>
#include <limits>

namespace Planning
{

    // 定义图节点：包含纵向索引和车道ID (0:左车道, 1:右车道)
    struct LaneNode {
        int index;
        int lane_id; 

        bool operator==(const LaneNode& other) const {
            return index == other.index && lane_id == other.lane_id;
        }
    };
    struct NodeDist {
        LaneNode node;
        double dist;
        bool operator>(const NodeDist& other) const {
            return dist > other.dist;
        }
    };
    class GlobalPlannerNormal : public GlobalPlannerBase
    {
    public:
        GlobalPlannerNormal();
        Path search_global_path(const PNCMap &pnc_map) override;

    private:
        double calc_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2);
    };
}

#endif