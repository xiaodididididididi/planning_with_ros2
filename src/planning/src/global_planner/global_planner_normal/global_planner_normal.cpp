#include "global_planner_normal.h"

namespace Planning
{
    GlobalPlannerNormal::GlobalPlannerNormal()
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_planner_normal created");

        global_planner_config_ = std::make_unique<ConfigReader>();
        global_planner_config_->read_global_path_config();
        global_planner_type_ = static_cast<int>(GlobalPlannerType::NORMAL);
    }

    Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "using Dijkstra global_planner");

        global_path_.header.frame_id = pnc_map.header.frame_id;
        global_path_.header.stamp = rclcpp::Clock().now();
        global_path_.poses.clear();

        // 1. 数据对齐与准备：计算左、右两条车道的中心点
        const int n = pnc_map.left_boundary.points.size();
        if (n < 2)
            return global_path_;

        std::vector<std::vector<geometry_msgs::msg::Point>> lane_centers(2, std::vector<geometry_msgs::msg::Point>(n));
        for (int i = 0; i < n; ++i)
        {
            // 左车道中心 (Lane 0): 左边界与中线的均值
            lane_centers[0][i].x = (pnc_map.left_boundary.points[i].x + pnc_map.midline.points[2 * i].x) / 2.0;
            lane_centers[0][i].y = (pnc_map.left_boundary.points[i].y + pnc_map.midline.points[2 * i].y) / 2.0;
            // 右车道中心 (Lane 1): 中线与右边界的均值
            lane_centers[1][i].x = (pnc_map.midline.points[2 * i].x + pnc_map.right_boundary.points[i].x) / 2.0;
            lane_centers[1][i].y = (pnc_map.midline.points[2 * i].y + pnc_map.right_boundary.points[i].y) / 2.0;
        }

        // 2. Dijkstra 算法
        std::vector<std::vector<double>> dist(2, std::vector<double>(n, std::numeric_limits<double>::infinity()));
        std::vector<std::vector<LaneNode>> prev(2, std::vector<LaneNode>(n, {-1, -1}));
        std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;

        // 起点：默认从右车道起点出发 (lane_id = 1)
        dist[1][0] = 0.0;
        pq.push({{0, 1}, 0.0});
        // 同时也把左车道起点加入，但给它一点初始代价，诱导算法选右边
        dist[0][0] = 5.0;
        pq.push({{0, 0}, 5.0});

        while (!pq.empty())
        {
            LaneNode u = pq.top().node;
            double d = pq.top().dist;
            pq.pop();

            if (d > dist[u.lane_id][u.index])
                continue;
            if (u.index == n - 1)
                continue; // 到达终点层

            // 向下一排（index + 1）扩展邻居
            int next_idx = u.index + 1;
            for (int next_lane : {0, 1})
            {
                double step_cost = calc_distance(lane_centers[u.lane_id][u.index], lane_centers[next_lane][next_idx]);

                // 增加逻辑权重
                if (u.lane_id != next_lane)
                {
                    step_cost += 10.0; // 变道惩罚系数：值越大越不容易变道
                }
                if (next_lane == 0)
                {
                    step_cost += 0.5; // 左车道行驶惩罚：值越大越偏向右车道
                }

                if (dist[u.lane_id][u.index] + step_cost < dist[next_lane][next_idx])
                {
                    dist[next_lane][next_idx] = dist[u.lane_id][u.index] + step_cost;
                    prev[next_lane][next_idx] = u;
                    pq.push({{next_idx, next_lane}, dist[next_lane][next_idx]});
                }
            }
        }

        // 3. 路径回溯 (取终点层代价最小的那个)
        int last_lane = (dist[1][n - 1] <= dist[0][n - 1]) ? 1 : 0;
        std::vector<LaneNode> path_nodes;
        for (LaneNode curr = {n - 1, last_lane}; curr.index != -1; curr = prev[curr.lane_id][curr.index])
        {
            path_nodes.push_back(curr);
        }
        std::reverse(path_nodes.begin(), path_nodes.end());

        // 4. 填充 ROS 消息
        for (const auto &node : path_nodes)
        {
            PoseStamped ps;
            ps.header = global_path_.header;
            ps.pose.position = lane_centers[node.lane_id][node.index];
            global_path_.poses.push_back(ps);
        }

        RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_path created, points size: %ld", global_path_.poses.size());
        return global_path_;
    }

    double GlobalPlannerNormal::calc_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }
}