#pragma once

#include <ros/ros.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <navfn/MakeNavPlan.h>
#include <dynamic_reconfigure/server.h>
#include <pure_voronoi_planner/boost_voronoi.h>
#include <pure_voronoi_planner/planning_utils.h>
#include <splined_voronoi/MakePlanWithStats.h>
#include <pure_voronoi_planner/PureVoronoiPlannerConfig.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <chrono>

namespace voronoi_planner
{


class PureVoronoiPlanner : public nav_core::BaseGlobalPlanner
{
public:
    PureVoronoiPlanner();
    PureVoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ~PureVoronoiPlanner();

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& res);

    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @remark New on MBF API
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    bool cancel();

private:
    bool initialized_;
    std::shared_ptr<costmap_2d::Costmap2D> costmap_;
    std::string costmap_global_frame_;
    cv::Point2d map_origin_;
    int costmap_size_x_;
    int costmap_size_y_;
    double costmap_resolution_;  // m / px
    double free_cell_threshold_;
    double min_radius_;
    bool use_dijkstra_;
    double free_space_factor_;
    bool large_free_spaces_;
    double time_taken_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_service_;
    ros::ServiceServer make_plan_with_stats_service_;
    ros::Publisher plan_pub_;
    ros::Publisher voronoi_map_pub_;

    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    cv::Point2i start_map_;
    cv::Point2i goal_map_;
    cv::Point2d start_world_;
    cv::Point2d goal_world_;

    cv::Mat voronoi_img_;
    cv::Mat obstacle_img_;

    dynamic_reconfigure::Server<pure_voronoi_planner::PureVoronoiPlannerConfig> *dsrv_;

    void reconfigureCB(pure_voronoi_planner::PureVoronoiPlannerConfig &config, uint32_t level);

    bool makePlanWithStatsService(splined_voronoi::MakePlanWithStats::Request& req, splined_voronoi::MakePlanWithStats::Response& res);

    /** @brief find complete path from start to goal; taken from https://github.com/frontw/voronoi_planner
     *
     * three components: start to voronoi; on voronoi; voronoi to goal
     *
     */
    bool findCompletePath(std::vector<cv::Point2i>& path, cv::Point2i start, cv::Point2i goal);

    /** @brief publishes Plan for visualization with rviz; adapted version of https://github.com/frontw/voronoi_planner
     *
     * @param which_path which publisher should be used: 0 -> original plan; 1 -> sparse plan; everything else ->
     * final plan
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
};
};  // namespace voronoi_planner
