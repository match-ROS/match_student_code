#include <pure_voronoi_planner/pure_voronoi_planner.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(voronoi_planner::PureVoronoiPlanner, nav_core::BaseGlobalPlanner)

namespace voronoi_planner
{

PureVoronoiPlanner::PureVoronoiPlanner()
{
    ROS_INFO("PureVoronoiPlanner empty Constructor got called!");
}

PureVoronoiPlanner::PureVoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("PureVoronoiPlanner Constructor got called!");
    initialize(name, costmap_ros);
}

PureVoronoiPlanner::~PureVoronoiPlanner()
{
    ROS_INFO("Deconstructor called");

    // if (grid_map_for_dynamicvoronoi_)
    // {
    //     for (int x = 0; x < this->costmap_size_x_; x++)
    //         delete[] grid_map_for_dynamicvoronoi_[x];
    //     delete[] grid_map_for_dynamicvoronoi_;
    // }
}

void PureVoronoiPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        ROS_INFO("PureVoronoiPlanner Init got called!");
        ROS_INFO_STREAM("Name: " << name);
        ros::NodeHandle private_nh("~/" + name);
        this->costmap_ = std::shared_ptr<costmap_2d::Costmap2D>(costmap_ros->getCostmap());
        this->costmap_global_frame_ = costmap_ros->getGlobalFrameID();
        this->costmap_size_x_ = costmap_->getSizeInCellsX();
        this->costmap_size_y_ = costmap_->getSizeInCellsY();
        this->costmap_resolution_ = costmap_->getResolution();
        this->map_origin_ = cv::Point2d(this->costmap_->getOriginX(), this->costmap_->getOriginY());

        ROS_INFO_STREAM("Costmap values:");
        ROS_INFO_STREAM("Size: " << this->costmap_size_x_ << ", " << this->costmap_size_y_);
        ROS_INFO_STREAM("Origin: " << this->map_origin_.x << ", " << this->map_origin_.y);
        ROS_INFO_STREAM("Resolution: " << this->costmap_resolution_);

        // get params
        private_nh.param("free_cell_threshold", this->free_cell_threshold_, 0.0);
        private_nh.param("min_radius", this->min_radius_, 0.0);
        private_nh.param("use_dijkstra", this->use_dijkstra_, false);
        private_nh.param("free_space_factor", this->free_space_factor_, 4.0);
        private_nh.param("large_free_spaces", this->large_free_spaces_, true);
        ROS_INFO_STREAM("Load param free_cell_threshold: " << this->free_cell_threshold_);
        ROS_INFO_STREAM("Load param min_radius: " << this->min_radius_);
        ROS_INFO_STREAM("Load param use_dijkstra: " << this->use_dijkstra_);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("/voronoi_planner/formation_plan", 1);
        voronoi_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("voronoi_map", 1);

        this->make_plan_service_ = private_nh.advertiseService("make_plan", &PureVoronoiPlanner::makePlanService, this);

        this->make_plan_with_stats_service_ = private_nh.advertiseService("make_plan_with_stats", &PureVoronoiPlanner::makePlanWithStatsService, this);

        dsrv_ = new dynamic_reconfigure::Server<pure_voronoi_planner::PureVoronoiPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<pure_voronoi_planner::PureVoronoiPlannerConfig>::CallbackType cb = boost::bind(
                &PureVoronoiPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }
    else
    {
        ROS_ERROR("VoronoiPlanner already initialized, aborting init!");
    }
}

void PureVoronoiPlanner::reconfigureCB(pure_voronoi_planner::PureVoronoiPlannerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    this->free_cell_threshold_ = config.free_cell_threshold;
    this->min_radius_ = config.min_radius;
    this->use_dijkstra_ = config.use_dijkstra;
    this->free_space_factor_ = config.free_space_factor;
    this->large_free_spaces_ = config.large_free_spaces;
}

bool PureVoronoiPlanner::cancel()
{
    ROS_ERROR("VoronoiPlanner CANCEL");
    // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    return false;
}

bool PureVoronoiPlanner::makePlanWithStatsService(splined_voronoi::MakePlanWithStats::Request& req, splined_voronoi::MakePlanWithStats::Response& res)
{
    std::vector<geometry_msgs::PoseStamped> path;

    req.start.header.frame_id = "map";
    req.goal.header.frame_id = "map";
    bool success = makePlan(req.start, req.goal, path);
    if (success)
    {
        res.plan_found = 0;
        res.time_taken = this->time_taken_;
        res.path = path;
    }
    else
    {
        res.plan_found = -1;
    }

    return true;
}

bool PureVoronoiPlanner::makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& res)
{
    std::vector<geometry_msgs::PoseStamped> path;

    req.start.header.frame_id = "map";
    req.goal.header.frame_id = "map";
    bool success = makePlan(req.start, req.goal, path);
    res.plan_found = success;
    if (success)
    {
        res.path = path;
    }

    return true;
}

bool PureVoronoiPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("VoronoiPlanner makePlan got called!");
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    this->start_ = start;
    this->goal_ = goal;

    // clear the plan, just in case
    plan.clear();

    std::string global_frame = costmap_global_frame_;

    // require the goal to be in our global frame
    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                  global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                  global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    this->start_world_ = cv::Point2d(start.pose.position.x, start.pose.position.y);

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;

    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x_i, start_y_i))
    {
        ROS_ERROR(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot "
            "has been properly localized?");
        return false;
    }
    this->start_map_.x = start_x_i;
    this->start_map_.y = start_y_i;

    this->goal_world_ = cv::Point2d(goal.pose.position.x, goal.pose.position.y);

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x_i, goal_y_i))
    {
        ROS_ERROR("The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    this->goal_map_.x = goal_x_i;
    this->goal_map_.y = goal_y_i;

    ROS_WARN_STREAM("Planning from " << this->start_world_ << " to " << this->goal_world_);

    std::chrono::steady_clock::time_point coordinate_transform_time = std::chrono::steady_clock::now();

    ROS_INFO_STREAM(
        "VoronoiPlanner: Time for start and goal coordinate transforms (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(coordinate_transform_time - begin).count()) /
               1000000.0);

    // TODO: move map to member variable to enable difference checking
    // Creating an empty bool array of matching size to the costmap
    costmap_size_x_ = costmap_->getSizeInCellsX();
    costmap_size_y_ = costmap_->getSizeInCellsY();

    cv::Mat costmap_img_orig = cv::Mat::zeros(costmap_size_x_, costmap_size_y_, CV_8UC1);
    int obstacle_count = 0;
    for (int i = 0; i < costmap_size_x_; i++)
    {
        for (int j = 0; j < costmap_size_y_; j++)
        {
            double cell_cost = costmap_->getCost(i, j);
            if (cell_cost > free_cell_threshold_)
            {
                costmap_img_orig.at<uchar>(i, j, 0) = 255;
                obstacle_count++;
            }
        }
    }
    // generate distance map
    cv::Mat costmap_img;
    cv::distanceTransform(~costmap_img_orig, costmap_img, cv::DIST_L2, 3, CV_8UC1);
    // threshold with formation radius
    cv::threshold(costmap_img, costmap_img, this->min_radius_ / this->costmap_resolution_ , 255, cv::THRESH_BINARY_INV);
    costmap_img.convertTo(costmap_img, CV_8UC1);

    obstacle_img_ = costmap_img;

    if (obstacle_img_.at<uchar>(start_map_.x, start_map_.y) == 255 || obstacle_img_.at<uchar>(goal_map_.x, goal_map_.y) == 255)
    {
        ROS_ERROR("Start or goal is inside obstacle. Planning will always fail. Aborting..");
        return false;
    }

    cv::Mat label_img;
    int num_labels = cv::connectedComponents(~obstacle_img_, label_img, 8);
    ROS_INFO_STREAM("Got " << num_labels << " connected components");

    if (label_img.at<int>(start_map_.x, start_map_.y) != label_img.at<int>(goal_map_.x, goal_map_.y))
    {
        ROS_ERROR("Start and goal are not in the same connected component; Planning will always fail. Aborting..");
        return false;
    }

    std::chrono::steady_clock::time_point costmap_as_img_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "VoronoiPlanner: Time taken for putting costmap in image (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(costmap_as_img_time - coordinate_transform_time)
                .count()) /
               1000000.0);
    cv::Mat canny_output;
    cv::Canny(costmap_img, canny_output, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat(canny_output.size(), CV_8UC3, cv::Scalar(255, 255, 255));
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(0, 0, 0);
        cv::drawContours(drawing, contours, (int)i, color, 1, cv::LINE_4, hierarchy, 0);
    }
    std::chrono::steady_clock::time_point img_contour_processing_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "VoronoiPlanner: Time taken for processing contours in image (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(img_contour_processing_time - costmap_as_img_time)
                .count()) /
               1000000.0);
    if (obstacle_count == 0)
    {
        ROS_WARN("No obstacles found in costmap, skipping voronoi generation.");
        voronoi_img_ = cv::Mat(costmap_size_x_, costmap_size_y_, CV_8UC1, cv::Scalar(255));
    }
    else
    {
        bool voronoi_creation_success = voronoi_generation::create_boost_voronoi(costmap_img, voronoi_img_, false);
        // cv::imwrite("/home/rosmatch/Bilder/voronoi_output.png", voronoi_img_);
    }

    std::chrono::steady_clock::time_point voronoi_generation_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "VoronoiPlanner: Time taken for generating voronoi image (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(voronoi_generation_time - img_contour_processing_time)
                .count()) /
               1000000.0);

    // generate distance map
    cv::Mat dist_img_narrow;
    cv::distanceTransform(~costmap_img_orig, dist_img_narrow, cv::DIST_L2, 3, CV_8UC1);
    // threshold with 4 times formation radius
    // double free_space_factor_ = 4.0;
    cv::threshold(dist_img_narrow, dist_img_narrow, free_space_factor_ * this->min_radius_ / this->costmap_resolution_ , 255, cv::THRESH_BINARY);
    dist_img_narrow.convertTo(dist_img_narrow, CV_8UC1);
    // bitwise or with voronoi image to combine both
    if (large_free_spaces_)
    {
        cv::bitwise_or(this->voronoi_img_, dist_img_narrow, this->voronoi_img_);
    }

    // Modifying the cost of the global costmap
    cv::Mat out_voronoi(costmap_size_x_, costmap_size_y_, CV_8UC1, cv::Scalar(0));
    // costmap_2d::Costmap2D voronoi_map(costmap_size_x_, costmap_size_y_, costmap_resolution_, costmap_->getOriginX(),
    //                                   costmap_->getOriginY(), 254);
    nav_msgs::OccupancyGrid voronoi_map;
    voronoi_map.header.frame_id = this->costmap_global_frame_;
    voronoi_map.header.stamp = ros::Time::now();
    voronoi_map.info.map_load_time = ros::Time::now();
    voronoi_map.info.height = this->costmap_size_y_;
    voronoi_map.info.width = this->costmap_size_x_;
    voronoi_map.info.resolution = this->costmap_resolution_;
    voronoi_map.info.origin.position.x = this->costmap_->getOriginX();
    voronoi_map.info.origin.position.y = this->costmap_->getOriginY();
    voronoi_map.data.assign(voronoi_map.info.width * voronoi_map.info.height, 100);
    // ROS_INFO("VoronoiPlanner created image!");
    for (int i = 0; i < costmap_size_x_; i++)
    {
        for (int j = 0; j < costmap_size_y_; j++)
        {
            // if (voronoi_.isVoronoi(i, j))
            int occupancy_index = j * costmap_size_x_ + i;
            if (voronoi_img_.at<uchar>(i, j) == 255)
            {
                out_voronoi.at<uchar>(i, j) = 255;
                voronoi_map.data[occupancy_index] = 0;
                // voronoi_map.setCost(i, j, 0);
            }
            else if (voronoi_img_.at<uchar>(i, j) > 0)
            {
                voronoi_map.data[occupancy_index] = 50;
            }
        }
    }
    std::chrono::steady_clock::time_point voronoi_map_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "VoronoiPlanner: Time taken for putting voronoi image in map (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(voronoi_map_time - voronoi_generation_time).count()) /
               1000000.0);

    cv::imwrite("/home/soenke/Bilder/voronoi_output.png", out_voronoi);
    voronoi_map_pub_.publish(voronoi_map);

    std::chrono::steady_clock::time_point voronoi_pub_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "VoronoiPlanner: Time taken for publishing voronoi map (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(voronoi_pub_time - voronoi_map_time).count()) /
               1000000.0);

    std::vector<cv::Point2i> path;
    bool path_found = this->findCompletePath(path, this->start_map_, this->goal_map_);
    std::chrono::steady_clock::time_point path_finding_time = std::chrono::steady_clock::now();
    double path_finding_secs = (std::chrono::duration_cast<std::chrono::microseconds>(path_finding_time - voronoi_pub_time).count()) / 1000000.0;
    // this->time_taken_ = path_finding_secs;
    ROS_INFO_STREAM("VoronoiPlanner: Time taken for finding path (s): " << path_finding_secs);

    if (!path_found && path.empty())
    {
        ROS_ERROR("No valid path found, returning empty plan");
        return false;
    }
    else if (!path_found)
    {
        ROS_WARN("Direct Connection between Start and Goal");
    }

    std::vector<cv::Point3d> path_world;
    plan_utils::pathMapToWorld(path, path_world, this->costmap_);

    std::vector<geometry_msgs::PoseStamped> out_plan;
    plan_utils::createPlanFromPath(path_world, plan, this->costmap_global_frame_);

    if (!plan_utils::isPlanFree(costmap_, free_cell_threshold_, out_plan))
    {
        ROS_ERROR("Plan is not free!");
        return false;
    }

    for (auto pose : out_plan)
    {
        plan.push_back(pose);
    }
    if (plan.empty())
    {
        ROS_ERROR("Got empty plan from spline interpolation");
        return false;
    }

    this->publishPlan(plan);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("VoronoiPlanner: Time taken for Postprcessing (s): "
                    << (std::chrono::duration_cast<std::chrono::microseconds>(end - path_finding_time).count()) /
                           1000000.0);
    double total_time_taken = (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0;
    ROS_INFO_STREAM("VoronoiPlanner: Total Time taken (s): "
                    << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0);

    return true;
}


bool PureVoronoiPlanner::findCompletePath(std::vector<cv::Point2i>& path, cv::Point2i start, cv::Point2i goal)
{
    ROS_INFO("inside findCompletePath");
    // std::vector<std::pair<int, int>> path1;
    // std::vector<std::pair<int, int>> path2;
    // std::vector<std::pair<int, int>> path3;

    // std::vector<cv::Point2i> path2;

    int connect_goal_success = PlanningStatus::Failed;
    int connect_start_success = PlanningStatus::Failed;

    // if goal is not on voronoi graph, compute shortest path to voronoi graph
    bool start_is_voronoi = voronoi_img_.at<uchar>(start.x, start.y) == 255;
    bool goal_is_voronoi = voronoi_img_.at<uchar>(goal.x, goal.y) == 255;
    std::chrono::steady_clock::time_point path_start_time = std::chrono::steady_clock::now();
    // if (!voronoi_.isVoronoi(goal_x, goal_y))
    cv::Point2i start_on_voronoi;
    cv::Point2i goal_on_voronoi;
    if (!goal_is_voronoi)
    {
        connect_goal_success = dijkstra_planning::findDijkstraPathToVoronoi(
            obstacle_img_, voronoi_img_, goal_on_voronoi, goal, start,
            false);
        // path3.push_back({goal_x, goal_y});
    }
    else
    {
        goal_on_voronoi = goal;
        connect_goal_success = PlanningStatus::Success;
    }
    std::chrono::steady_clock::time_point path_goal_connection_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "PathFinding: Time taken for goal connection (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(path_goal_connection_time - path_start_time)
                .count()) /
               1000000.0);

    // if start is not on voronoi graph, compute shortest path to voronoi graph
    // if (!voronoi_.isVoronoi(start_x, start_y))
    if (!start_is_voronoi)
    {
        connect_start_success = dijkstra_planning::findDijkstraPathToVoronoi(
            obstacle_img_, voronoi_img_, start_on_voronoi, start, goal,
            false);
    }
    else
    {
        start_on_voronoi = start;
        connect_start_success = PlanningStatus::Success;
    }

    if (connect_start_success == PlanningStatus::DirectConnectionToGoal || connect_goal_success == PlanningStatus::DirectConnectionToGoal)
    {
        // skip planning and only use start and goal
        path.push_back(start);
        path.push_back(goal);
    }

    if (!(connect_start_success == PlanningStatus::Success && connect_goal_success == PlanningStatus::Success))
    {
        ROS_ERROR("Failed to connect start or goal to voronoi");
        return false;
    }

    std::chrono::steady_clock::time_point path_start_connection_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("PathFinding: Time taken for start connection (s): "
                    << (std::chrono::duration_cast<std::chrono::microseconds>(path_start_connection_time -
                                                                              path_goal_connection_time)
                            .count()) /
                           1000000.0);
    // find path on voronoi
    bool on_voronoi_success = false;
    if (this->use_dijkstra_)
    {
        on_voronoi_success = dijkstra_planning::findDijkstraPathOnVoronoi(voronoi_img_, path, start_on_voronoi,  goal_on_voronoi, true);
    }
    else
    {
        on_voronoi_success = astar_planning::findAStarPathOnImage(voronoi_img_, path, start_on_voronoi, goal_on_voronoi, true);
    }

    std::chrono::steady_clock::time_point path_complete_time = std::chrono::steady_clock::now();
    double path_on_voronoi_secs = (std::chrono::duration_cast<std::chrono::microseconds>(path_complete_time - path_start_connection_time).count()) / 1000000.0;
    this->time_taken_ = path_on_voronoi_secs;
    ROS_INFO_STREAM(
        "PathFinding: Time taken for on voronoi connection (s): "
        << path_on_voronoi_secs);

    if (!on_voronoi_success)
    {
        ROS_ERROR("Failed to find path on voronoi");
        return false;
    }

    // fuse all paths to single vector
    // dont use path to start as it is better for formation to turn to start and linear move and turn again
    // dont use path to end as later the goal will be added manually and sparse path doesn't need further processing
    // path.insert(path.end(), path1.begin(), path1.end());
    // path.insert(path.end(), path2.begin(), path2.end());
    // path.insert(path.end(), path3.begin(), path3.end());
    return true;
}

void PureVoronoiPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = ros::Time::now();
    }
    else
    {
        ROS_WARN("Got empty plan");
        return;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

};  // namespace voronoi_planner