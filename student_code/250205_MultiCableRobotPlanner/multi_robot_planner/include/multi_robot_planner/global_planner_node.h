#ifndef GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <string>
#include <multi_robot_planner/StartGoalPair.h>
#include <multi_robot_planner/StartGoalPairList.h>
#include <multi_robot_planner/ScoringMetric.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <memory>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <unordered_set>
#include <utility>
#include <functional>
#include <algorithm>
#include <random>
#include <ctime>

// Struct for the representation of a node in the path planning algorithm
struct Node {
    int x, y;
    double g, h;
    Node* parent;
    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    double f() const { return g + h; }
};

// Comparison operator for the priority queue
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) {
        return a->f() > b->f();
    }
};

// Definition of a hash function for std::pair<int, int>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

// Struct for the storage of all data for one start goal pair
struct RobotData {
    int robot_id;
    int stage;
    multi_robot_planner::StartGoalPair start_goal_pair;
    std::vector<std::pair<int, int>> path;
    std::vector<std::pair<int, int>> cable_path;
    nav_msgs::Path gui_path;
    nav_msgs::Path gui_cable_path;
    std::vector<std::pair<int, int>> path_costmap_points;
    std::unordered_set<std::pair<int, int>, pair_hash> path_costmap_points_set;
    std::vector<std::pair<int, int>> start_points;
    std::vector<std::pair<int, int>> goal_points;
    std::pair<int, int> cable_start;
    std::pair<int, int> cable_goal;
    std::vector<std::pair<int, int>> cable_start_points;
    std::vector<std::pair<int, int>> cable_goal_points;
    double path_length;
    double cable_length;
    double distance;
    bool ripped_up;
};

// Struct for sorting paths by cable lengths
struct RobotCableLengthSorted {
    int index;
    double length;
    int robot_id;
};

// Struct for sorting paths by length ratios
struct LengthRatioSorted {
    int index;
    double ratio;
};

// Struct for sorting paths by distance from isolated nodes (Applied in Rip up closest)
struct PathDistanceToIsolatedNode {
    int index;
    double distance;
    int robot_id;
};

// Struct for sorting paths by cost (Applied in Rip up by cost)
struct PathCost {
    int index;
    double cost;
};

class GlobalPlanner {
    ros::Publisher ready_pub_;
    bool planner_ready_;
    bool pairs_ready_;
    
public:
    GlobalPlanner(ros::NodeHandle& nh);
    void waitForCostmap();
    void processPairs();
    std::vector<std::pair<int, int>> astar();
    void publishPath(const nav_msgs::Path gui_path, int robot_id);
    void publishCablePath(const std::vector<std::pair<int, int>>& path, int robot_id);
    void publishScoringMetric();
    std::vector<multi_robot_planner::StartGoalPair> getStartGoalPairs() const { return start_goal_pairs_; }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher costmap_pub_;
    ros::Publisher costmap_static_pub_;
    std::vector<ros::Publisher> costmap_pubs_;
    std::vector<ros::Publisher> costmap_path_pubs_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber pairs_sub_;
    std::vector<ros::Publisher> path_publishers_;
    std::vector<ros::Publisher> cable_path_publishers_;
    ros::Publisher feedback_pub_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid costmap_;
    std::vector<nav_msgs::OccupancyGrid> costmaps_;
    std::vector<nav_msgs::OccupancyGrid> costmaps_path_;
    nav_msgs::OccupancyGrid costmap_static_;

    std::vector<std::pair<int, int>> costmap_obstacles_;
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    bool start_received_ = false;
    bool goal_received_ = false;
    std::vector<multi_robot_planner::StartGoalPair> start_goal_pairs_;
    bool processing_done_;
    int pair_nr;
    std::vector<std::pair<int, int>> path_costmap_points_;
    std::unordered_set<std::pair<int, int>, pair_hash> path_costmap_points_set_;
    std::map<int, std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::ExePathAction>>> action_clients_;
    int paths_planned_;
    std::vector<int> robot_order_;
    std::map<int, std::vector<int>> robot_order_map_;
    
    int costmap_width_;
    int costmap_height_;
    float inflation_radius_robot_;
    float inflation_radius_cable_;
    float costmap_resolution_;
    int cells_radius_robot_;
    int cells_radius_cable_;
    double origin_x_;
    double origin_y_;
    bool allow_crossing_;
    std::vector<std::tuple<int, int, int>> dominant_paths_;
    int robot_quantity_;
    bool failed_;

    ros::Publisher scoring_metric_pub_;
    multi_robot_planner::ScoringMetric scoring_metric_;
    int pair_quantity_;
    int ripped_up_nr_;
    float completion_rate_;
    int pairs_failed_nr_;
    int failed_routed_nr_;
    int rerouted_nr_;
    float routability_;
    float reroutability_;
    float computing_time_;
    float cable_length_;
    float cable_length_ratio_;
    float path_length_;
    float path_length_ratio_;
    
    std::vector<RobotData> robot_data_list_;
    std::vector<std::vector<RobotData>> robot_data_list_stages_;
    std::vector<RobotCableLengthSorted> robot_cable_lengths_sorted_;
    std::vector<LengthRatioSorted> robot_length_ratios_sorted_;
    std::vector<PathDistanceToIsolatedNode> robot_path_distance_to_node_;
    std::vector<PathCost> robot_path_cost_;
    
    std::pair<int, int> worldToMap(double wx, double wy);
    std::pair<double, double> mapToWorld(int mx, int my) const;
    void createGUIPath(const std::vector<std::pair<int, int>>& path);
    void preSortPairsByStage();
    void preSortPairsByLength();
    void preSortPairsCustom();
    void printPlanningOrder();
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void markInflationZone(int cell_x, int cell_y, int radius);
    void pairsCallback(const multi_robot_planner::StartGoalPairList::ConstPtr& msg);
    void updatePoint(int x, int y, int robot_id, int cost_value);
    void updatePointPath(int x, int y, int robot_id, int cost_value);
    void updateCostmapWithPath(int pair_index);
    void measureCableLength(int path_index);
    void updateCostmapsWithStartGoal();
    void ripUpRandom();
    void ripUpByLength();
    void ripUpByLengthRatio();
    void ripUpClosest();
    void ripUpByCost();
    void ripUpCrossedPaths();
    void ripUpPath(int path_index);
    void initializeActionClients(const std::vector<int>& robot_ids);
    void sendAllPathsToRobots();
    void sendPathToRobot(int robot_id, const nav_msgs::Path& path);
};

#endif // GLOBAL_PLANNER_NODE_H

