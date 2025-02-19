#include "multi_robot_planner/global_planner_node.h"

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh)
  : nh_(nh),
    planner_ready_(false),
    pairs_ready_(false),
    processing_done_(false),
    pair_nr(1)
{
    // Initialize publishers and subscribers
    map_sub_ = nh.subscribe("/map", 1, &GlobalPlanner::costmapCallback, this);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_planner_costmap", 1);
    costmap_static_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_planner_costmap_static", 1);
    pairs_sub_ = nh.subscribe("/start_goal_pairs", 1, &GlobalPlanner::pairsCallback, this);
    feedback_pub_ = nh_.advertise<std_msgs::Bool>("/global_planner/feedback", 1);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/global_planner/ready", 1);
    scoring_metric_pub_ = nh.advertise<multi_robot_planner::ScoringMetric>("/scoring_metric", 100);
    ROS_INFO("GlobalPlanner Node initialized.");
}

void GlobalPlanner::waitForCostmap() {
    ros::Rate rate(10);  // 10 Hz
    while (costmap_.data.empty()) {
        ROS_INFO("Waiting for costmap...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Costmap received, size: %dx%d", costmap_.info.width, costmap_.info.height);
}

double getYawFromPose(const geometry_msgs::PoseStamped& pose)
{
    const geometry_msgs::Quaternion& q = pose.pose.orientation;

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    return yaw;
}

std::pair<int, int> GlobalPlanner::worldToMap(double wx, double wy) {
    // Convert world coordinates to map coordinates
    int mx = static_cast<int>(std::round((wx - origin_x_) / costmap_resolution_));
    int my = static_cast<int>(std::round((wy - origin_y_) / costmap_resolution_));
    return std::make_pair(mx, my);
}

std::pair<double, double> GlobalPlanner::mapToWorld(int mx, int my) const {
    // Convert map coordinates to world coordinates
    double wx = mx * costmap_resolution_ + origin_x_;
    double wy = my * costmap_resolution_ + origin_y_;
    return std::make_pair(wx, wy);
}

void GlobalPlanner::processPairs() {
    ros::Duration(2.0).sleep();
    
    // Wair for pairs_ready_ flag
    ros::Rate loop_rate(1);
    while (ros::ok() && !pairs_ready_) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check if processing is done
    if (processing_done_) {
        return;
    }
    
    // Set variables to 0
    paths_planned_ = 0;
    pairs_failed_nr_ = 0;
    failed_routed_nr_ = 0;
    rerouted_nr_ = 0;
    completion_rate_ = 0.0;
    routability_ = 0.0;
    reroutability_ = 0.0;
    cable_length_ = 0.0;
    cable_length_ratio_ = 0.0;
    path_length_ = 0.0;
    path_length_ratio_ = 0.0;
    failed_ = false;

    // Resize robot order vector
    robot_order_.resize(start_goal_pairs_.size());
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_[i] = i + 1;
    }

    // Make sure crossing existing paths are not allowed to be crossed on the first path planning attempt
    allow_crossing_ = false;

    initializeActionClients(robot_order_);
    
    // Presort robot data list by custom order
    //preSortPairsCustom();

    // Presort robot data list by stage and by length
    //preSortPairsByStage();
    //preSortPairsByLength();

    printPlanningOrder();

    // Update costmaps with the inflation zones of the obstacles caused by robots at their start and goal positions
    updateCostmapsWithStartGoal();

    // Start timer for computing time
    ros::Time start_time = ros::Time::now();
    computing_time_ = (ros::Time::now() - start_time).toSec();

    publishScoringMetric();
    
    // Process the path planning problem pair by pair
    ROS_INFO("Processing %ld start-goal pairs.", start_goal_pairs_.size());
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        
        // Print information about current start goal pair
        ROS_INFO("Processing pair %ld (stage %d, robot %d): Start (%f, %f), Goal (%f, %f)",
                i+1, robot_data_list_[i].stage, robot_data_list_[i].robot_id,
                robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
                robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
        
        // Reset the cost of dominant paths
        for (size_t j = 0; j < dominant_paths_.size(); ++j) {
            if (std::get<0>(dominant_paths_[j]) <= 0) {
                for (int k = 0; k < paths_planned_; ++k){
                    if (robot_data_list_[k].stage == std::get<1>(dominant_paths_[j])
                        && robot_data_list_[k].robot_id == std::get<2>(dominant_paths_[j])) {
                            ROS_INFO("Lowering cost of previous path.");
                            updateCostmapWithPath(k);
                    }
                }
            }
        }

        // Remove dominant paths from list
        for (size_t j = 0; j < dominant_paths_.size(); ++j) {
            if (std::get<0>(dominant_paths_[j]) <= 0) {
                for (int k = 0; k < paths_planned_; ++k){
                    if (robot_data_list_[k].stage == std::get<1>(dominant_paths_[j])
                        && robot_data_list_[k].robot_id == std::get<2>(dominant_paths_[j])
                        && std::get<0>(dominant_paths_[j]) <= 0) {
                            dominant_paths_.erase(dominant_paths_.begin() + j);
                    }
                }
            }
        }
        
        // Store start and goal poses
        start_ = robot_data_list_[i].start_goal_pair.start;
        goal_ = robot_data_list_[i].start_goal_pair.goal;

        // Attempt to find a path for the current start goal pair using A*
        auto path = astar();
        
        // Check if the path planner could not find a path
        if (path.empty()) {
            ROS_WARN("No path found for pair %ld (stage %d, robot %d).", i+1, robot_data_list_[i].stage, robot_data_list_[i].robot_id);
            
            // Update variables for the scoring metric
            routability_ = static_cast<float>(failed_routed_nr_) / pairs_failed_nr_;
            pairs_failed_nr_ += 1;
            if (robot_data_list_[pair_nr - 1].ripped_up == true) {
                robot_data_list_[pair_nr - 1].ripped_up = false;
            }

            // Check if there are no paths planned
            if (paths_planned_ == 0) {
                ROS_WARN("There is no feasible solution for robot %d to reach the goal point at (%f, %f)",
                        robot_order_[i],
                        robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
                pair_nr = start_goal_pairs_.size() + 1;
            }
            
            // Apply rip up and retry
            else {
                // Rip up one or more paths by applying a rip up strategy (Only one of the following strategies should be chosen)
                //ripUpRandom();
                //UpByLength();
                //ripUpByLengthRatio();
                //ripUpClosest();
                //ripUpByCost();
                //allow_crossing_ = true;  // This allows the current path to cross other paths and thereby applies the strategy Rip up crossed
                
                // Check if allow crossing is activated. If so: plan current path while allowing it to cross other paths and rip up crossed paths afterwards
                if (allow_crossing_ == true) {
                    ROS_INFO("Trying to find path with crossings.");

                    // Attempt to find a path for the current start goal pair using A*
                    auto path = astar();

                    // Check if the path planner could not find a path
                    if(path.empty()) {
                        ROS_WARN("No path with crossings found for pair %ld (stage %d, robot %d).", i+1, robot_data_list_[i].stage, robot_data_list_[i].robot_id);
                        
                        // Update variables for scoring metric
                        routability_ = static_cast<float>(failed_routed_nr_) / pairs_failed_nr_;
                        pairs_failed_nr_ += 1;

                        // Deactivate allowance for crossing paths
                        allow_crossing_ = false;

                        // Reset the cost of dominant paths
                        for (size_t j = 0; j < dominant_paths_.size(); ++j) {
                            for (size_t k = 0; k < robot_data_list_.size(); ++k){
                                if (robot_data_list_[k].stage == std::get<1>(dominant_paths_[j])
                                    && robot_data_list_[k].robot_id == std::get<2>(dominant_paths_[j])) {
                                        ROS_INFO("Lowering cost values of previous path.");
                                        updateCostmapWithPath(k);
                                        dominant_paths_.erase(dominant_paths_.begin() + j);
                                }
                            }
                        }
                        
                        // Try to process the same pair again with the cost of dominant paths lowered
                        --i;
                        continue;
                    }

                    // Update dominant paths
                    for (size_t j = 0; j < dominant_paths_.size(); ++j) {
                        std::get<0>(dominant_paths_[j]) -= 1;
                    }
                    
                    ROS_INFO("Path with crossings found with %ld waypoints.", path.size());

                    // Store path in robot data list
                    robot_data_list_[i].path = path;

                    // Create GUI path for publisher
                    createGUIPath(path);

                    // Update the costmap with the obstacle of the path
                    updateCostmapWithPath(pair_nr - 1);

                    // Add all single paths of the current robot to one gui path
                    nav_msgs::Path gui_path_robot;
                    std::vector<std::pair<int, int>> cable_path_robot;
                    std::vector<std::pair<int, int>> robot_indexes;
                    for (size_t i = 0; i < robot_data_list_.size(); ++i) {
                        if (robot_data_list_[i].robot_id == robot_data_list_[pair_nr - 1].robot_id) {
                            robot_indexes.push_back(std::make_pair(i, robot_data_list_[i].stage));
                        }
                    }
                    std::sort(robot_indexes.begin(), robot_indexes.end(), 
                        [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                            return a.second < b.second;
                        }
                    );
                    gui_path_robot = robot_data_list_[i].gui_path;
                    gui_path_robot.poses.clear();
                    for (size_t i = 0; i < robot_indexes.size(); ++i) {
                        gui_path_robot.poses.insert(gui_path_robot.poses.end(), robot_data_list_[robot_indexes[i].first].gui_path.poses.begin(), robot_data_list_[robot_indexes[i].first].gui_path.poses.end());
                        cable_path_robot.insert(cable_path_robot.end(), robot_data_list_[robot_indexes[i].first].cable_path.begin(), robot_data_list_[robot_indexes[i].first].cable_path.end());
                    }

                    publishPath(gui_path_robot, robot_data_list_[i].robot_id);  // Publish path on /robotN/global_path/robot_id
        
                    robot_data_list_[i].path_costmap_points_set = path_costmap_points_set_; //  Store the costmap points of the path
                    
                    measureCableLength(pair_nr - 1);
            
                    publishCablePath(cable_path_robot, robot_data_list_[i].robot_id);
                    
                    // Check if the path has been ripped up before and update the rerouted counter
                    if (robot_data_list_[i].ripped_up == true) {
                        rerouted_nr_ += 1;
                        robot_data_list_[i].ripped_up = false;
                    }

                    // Update counters
                    ++paths_planned_;
                    failed_routed_nr_ += 1;

                    routability_ = static_cast<float>(failed_routed_nr_) / pairs_failed_nr_; // Compute overall routability
                    
                    // Rip up all paths that are crossed by the new path
                    ripUpCrossedPaths();

                    allow_crossing_ = false;  // Deactivate path crossing
                }

                // Update current pair to failed if allow crossing is not active
                else {
                    failed_ = true;
                }
            }
            
            // Check if the robot order is already in the map
            for (const auto& pair : robot_order_map_) {
                if (pair.second == robot_order_) {
                    ROS_WARN("This order has already been tried before. Ripping up more paths");
                    
                    // Rip up one or more paths by applying a rip up strategy (Only one of the following strategies should be chosen)
                    //ripUpRandom();
                    //ripUpByLength();
                    //ripUpByLengthRatio();
                    //ripUpClosest();
                    //ripUpByCost();
                }
            }
            
            robot_order_map_[robot_order_map_.size()] = robot_order_;  // Update robot order map with current order
            
            // Print new robot order
            std::stringstream ss;
            ss << "New robot order: [";
            for (size_t i = 0; i < robot_order_.size(); ++i) {
                ss << robot_order_[i];
                if (i != robot_order_.size() - 1) {
                    ss << ", ";
                } else {
                    ss << "]";
                }
            }
            ROS_INFO("%s", ss.str().c_str());
            
            i = pair_nr - 2;

        // If the path planner found a path
        } else {
            // Update dominant paths
            for (size_t j = 0; j < dominant_paths_.size(); ++j) {
                std::get<0>(dominant_paths_[j]) -= 1;
            }

            ROS_INFO("Path found with %ld waypoints.", path.size());

            // Check if planning of the current path has failed before
            if (failed_ == true) {
                failed_routed_nr_ += 1;
            }
            failed_ = false;
            routability_ = static_cast<float>(failed_routed_nr_) / pairs_failed_nr_;  // Calculate overall routability

            robot_data_list_[i].path = path;  // Store current path in the robot data list
            
            createGUIPath(path);  // Create GUI path for publisher

            updateCostmapWithPath(pair_nr - 1);  // Update the costmap with the obstacles of the current path
            robot_data_list_[i].path_costmap_points_set = path_costmap_points_set_;  // Store the costmap points of the path

            // Add all single paths of the current robot to one gui path
            nav_msgs::Path gui_path_robot;
            std::vector<std::pair<int, int>> cable_path_robot;
            std::vector<std::pair<int, int>> robot_indexes;
            for (size_t i = 0; i < robot_data_list_.size(); ++i) {
                if (robot_data_list_[i].robot_id == robot_data_list_[pair_nr - 1].robot_id) {
                    robot_indexes.push_back(std::make_pair(i, robot_data_list_[i].stage));
                }
            }
            std::sort(robot_indexes.begin(), robot_indexes.end(), 
                [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                    return a.second < b.second;
                }
            );
            gui_path_robot = robot_data_list_[i].gui_path;
            gui_path_robot.poses.clear();
            for (size_t i = 0; i < robot_indexes.size(); ++i) {
                gui_path_robot.poses.insert(gui_path_robot.poses.end(), robot_data_list_[robot_indexes[i].first].gui_path.poses.begin(), robot_data_list_[robot_indexes[i].first].gui_path.poses.end());
                cable_path_robot.insert(cable_path_robot.end(), robot_data_list_[robot_indexes[i].first].cable_path.begin(), robot_data_list_[robot_indexes[i].first].cable_path.end());
            }
            
            publishPath(gui_path_robot, robot_data_list_[i].robot_id);  // Publish path on /robotN/global_path

            measureCableLength(pair_nr - 1);  // Measure the cable length for the current path

            publishCablePath(cable_path_robot, robot_data_list_[i].robot_id);  // Publish the cable path

            // Check if the current path has been ripped up before and update the rerouted counter
            if (robot_data_list_[pair_nr - 1].ripped_up == true) {
                rerouted_nr_ += 1;
                robot_data_list_[pair_nr - 1].ripped_up = false;
            }

            // Update counters and deactivate allow crossing
            ++paths_planned_;
            ++pair_nr;
            allow_crossing_ = false;
        }

        // Update scoring metric variables
        completion_rate_ = static_cast<float>(paths_planned_) / pair_quantity_;
        reroutability_ = static_cast<float>(rerouted_nr_) / ripped_up_nr_;
        cable_length_ = 0.0;
        path_length_ = 0.0;
        double distance_sum = 0.0;
        for (int i = 0; i < paths_planned_; ++i) {
            path_length_ += robot_data_list_[i].path_length;
            cable_length_ += robot_data_list_[i].cable_length;
            distance_sum += robot_data_list_[i].distance;
        }
        cable_length_ratio_ = cable_length_ / distance_sum;
        path_length_ratio_ = path_length_ / distance_sum;

        computing_time_ = (ros::Time::now() - start_time).toSec();

        publishScoringMetric();
    }
    
    // Check if all paths are planned
    if (paths_planned_ == start_goal_pairs_.size()) {
        sendAllPathsToRobots();  // Send the global paths to the robots
    }
    
    processing_done_ = true;

    std_msgs::Bool feedback_msg;
    feedback_msg.data = true;
    feedback_pub_.publish(feedback_msg);
    ros::Duration(0.1).sleep();
}

std::vector<std::pair<int, int>> GlobalPlanner::astar() {
    // Check if the costmap or start/goal points are missing
    if (costmap_.data.empty() || start_goal_pairs_.empty()) {
        ROS_WARN("Costmap or start/goal points not received, cannot perform A*.");
        return {};
    }
    
    ROS_INFO("Starting A* algorithm...");

    // Lambda function to check if a grid cell is valid (not an obstacle and within bounds)
    auto isValid = [&](int x, int y) {
        // Check if grid cell is within bounds
        if (x < 0 || x >= costmap_width_ || y < 0 || y >= costmap_height_) {
            return false;
        }
        if (costmap_static_.data[y * costmap_width_ + x] >= 50) {
            return false;
        }

        // Check for obstacles from other robots' costmaps
        for (int i = 0; i < robot_quantity_; ++i) {
            if (i + 1 == robot_data_list_[pair_nr - 1].robot_id) {
                continue;
            } else if (costmaps_[i].data[y * costmap_width_ + x] >= 50) {
                return false;
            }
        }
        
        // Additional checks for paths that might be crossing
        for (int i = 0; i < robot_quantity_; ++i) {
            if (i + 1 == robot_data_list_[pair_nr - 1].robot_id
                && costmaps_[i].data[y * costmap_width_ + x] >= 50) {
                    continue;
            } else {
                if (allow_crossing_ == false && costmaps_path_[i].data[y * costmap_width_ + x] >= 50) {
                    return false;
                }

                if (allow_crossing_ == true && costmaps_path_[i].data[y * costmap_width_ + x] >= 85) {
                    return false;
                }
            }
        }

        return true;  // Cell is valid if no conditions failed
    };

    // Lambda function for heuristic (Euclidean distance)
    auto heuristic = [&](int x, int y, int goal_x, int goal_y) {
        return std::hypot(goal_x - x, goal_y - y);  // Euclidian distance as heuristic
    };

    // Convert start and goal positions from world to map coordinates
    std::pair<int, int> start_map = worldToMap(start_.pose.position.x, start_.pose.position.y);
    int start_x = start_map.first;
    int start_y = start_map.second;

    std::pair<int, int> goal_map = worldToMap(goal_.pose.position.x, goal_.pose.position.y);
    int goal_x = goal_map.first;
    int goal_y = goal_map.second;
    
    ROS_INFO("Mapped start point to grid: (%d, %d)", start_map.first, start_map.second);
    ROS_INFO("Mapped goal point to grid: (%d, %d)", goal_map.first, goal_map.second);
    
    // Check if start and goal points are valid
    if (!isValid(start_map.first, start_map.second)) {
        ROS_WARN("Invalid start point on costmap.");
        return {};
    }

    if (!isValid(goal_map.first, goal_map.second)) {
        ROS_WARN("Invalid goal point on costmap.");
        return {};
    }

    // Initialize the open set (priority queue) and all nodes map
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;
    std::unordered_map<int, Node*> all_nodes;

    // Create the start node and add it to the open set
    Node* start_node = new Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
    open_set.push(start_node);
    all_nodes[start_y * costmap_width_ + start_x] = start_node;

    // Directions for moving (8 directions)
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                                   {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    // A* loop: expand nodes until the goal is reached
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();

        // Check if goal is reached
        if (current->x == goal_x && current->y == goal_y) {
            std::vector<std::pair<int, int>> path;
            while (current != nullptr) {
                path.emplace_back(current->x, current->y);  // Add current node to the path
                current = current->parent;  // Trace back to start node
            }
            std::reverse(path.begin(), path.end());  // Reverse the path to start -> goal

            // Clean up node memory
            for (auto& node : all_nodes) {
                delete node.second;
            }

            return path;  // Return the found path
        }

        // Expand neighbors (adjacent cells)
        for (auto& dir : directions) {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;

            if (!isValid(new_x, new_y)) {
                continue;  // Skip invalid neighbors
            }

            double g_new = current->g + std::hypot(dir.first, dir.second);
            double h_new = heuristic(new_x, new_y, goal_x, goal_y);

            int index = new_y * costmap_width_ + new_x;
            if (all_nodes.find(index) == all_nodes.end()) {
                Node* neighbor = new Node(new_x, new_y, g_new, h_new, current);
                open_set.push(neighbor);  // Add neighbor to open set
                all_nodes[index] = neighbor;
            } else if (g_new < all_nodes[index]->g) {
                all_nodes[index]->g = g_new;  // Update cost if a better path is found
                all_nodes[index]->parent = current;
            }
        }
    }

    // Clean up nodes if no path was found
    for (auto& node : all_nodes) {
        delete node.second;
    }

    return {};  // Return an empty path if no valid path was found
}

void GlobalPlanner::createGUIPath(const std::vector<std::pair<int, int>>& path) {
    // Create a new Path message to store the GUI path
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Convert each path point from grid coordinates to world coordinates and add it to the GUI path
    for (const auto& p : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        std::pair<double, double> world_coords = mapToWorld(p.first, p.second);
        pose.pose.position.x = world_coords.first;
        pose.pose.position.y = world_coords.second;
        pose.pose.orientation.w = 1.0;

        gui_path.poses.push_back(pose);
    }
    
    // Retrieve the goal from the start-goal pair
    const auto& goal = robot_data_list_[pair_nr - 1].start_goal_pair.goal;
    
    // Set the orientation of the last point in the path to match the goal orientation
    if (!gui_path.poses.empty()) {
        geometry_msgs::PoseStamped& last_pose = gui_path.poses.back();
        last_pose.pose.orientation = goal.pose.orientation;
    }

    // Store the GUI path in the robot data list
    robot_data_list_[pair_nr - 1].gui_path = gui_path;
}

void GlobalPlanner::preSortPairsByStage() {
    ROS_INFO("Presorting pairs by stage.");
    
    // Lambda function for calculating the euclidean distance between two geometry_msgs::PoseStamped
    auto calculateDistance = [](const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) -> double {
        double dx = goal.pose.position.x - start.pose.position.x;
        double dy = goal.pose.position.y - start.pose.position.y;
        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    };

    ROS_INFO("Calculating distances between start and goal points.");
    // Calculate the distance between start and goal for every pair
    for (size_t i = 0; i < robot_data_list_.size(); ++i) {
        robot_data_list_[i].distance = calculateDistance(robot_data_list_[i].start_goal_pair.start, robot_data_list_[i].start_goal_pair.goal);
    }

    ROS_INFO("Checking for the maximum stage.");
    // Check for the maximum stage and resize the vector robot_data_list_stages_ to the number of stages
    auto max_it = std::max_element(robot_data_list_.begin(), robot_data_list_.end(),
        [](const RobotData& a, const RobotData& b) {
            return a.stage < b.stage;
        }
    );
    int stage_max = (max_it != robot_data_list_.end()) ? max_it->stage : 0;
    robot_data_list_stages_.resize(stage_max);

    ROS_INFO("Creating the vector robot_data_list_stages_.");
    // Add the pairs to the vector robot_data_list_stages_ ordered by stages
    for (size_t i = 0; i < robot_data_list_.size(); ++i) {
        int stage = robot_data_list_[i].stage;
        ROS_INFO("Adding pair %ld to stage %d.", i+1, stage);
        robot_data_list_stages_[stage-1].push_back(robot_data_list_[i]);
    }

    ROS_INFO("Sorting robot data list by stage number.");
    // Sort the robot data list by stage number
    std::sort(robot_data_list_.begin(), robot_data_list_.end(), 
        [](const RobotData& a, const RobotData& b) {
            return a.stage < b.stage;
        }
    );
    ROS_INFO("Sorted robot data list by stage number.");
}

void GlobalPlanner::preSortPairsByLength() {
    ROS_INFO("Presorting pairs by length.");

    // Sort the pairs of each stage by distance
    for (int i = 0; i < robot_data_list_stages_.size(); ++i) {
        std::sort(robot_data_list_stages_[i].begin(), robot_data_list_stages_[i].end(),
            [](const RobotData& a, const RobotData& b) {
                return a.distance < b.distance;
            }
        );
    }

    // Sort the robot data list by stage number and then by distance
    robot_data_list_.clear();
    for (int i = 0; i < robot_data_list_stages_.size(); ++i) {
        for (int j = 0; j < robot_data_list_stages_[i].size(); ++j) {
            robot_data_list_.push_back(robot_data_list_stages_[i][j]);
        }
    }
}

void GlobalPlanner::preSortPairsCustom() {
    // Initialize a custom robot data list
    std::vector<RobotData> robot_data_list_custom;
    robot_data_list_custom.resize(start_goal_pairs_.size());

    // Assign robot data to new places in the custom robot data list
    robot_data_list_custom[0] = robot_data_list_[0];
    robot_data_list_custom[1] = robot_data_list_[1];
    robot_data_list_custom[2] = robot_data_list_[2];
    robot_data_list_custom[3] = robot_data_list_[6];
    robot_data_list_custom[4] = robot_data_list_[7];
    robot_data_list_custom[5] = robot_data_list_[3];
    robot_data_list_custom[6] = robot_data_list_[4];
    robot_data_list_custom[7] = robot_data_list_[5];
    robot_data_list_custom[8] = robot_data_list_[11];
    robot_data_list_custom[9] = robot_data_list_[10];
    robot_data_list_custom[10] = robot_data_list_[9];
    robot_data_list_custom[11] = robot_data_list_[8];
    robot_data_list_custom[12] = robot_data_list_[12];
    robot_data_list_custom[13] = robot_data_list_[13];
    robot_data_list_custom[14] = robot_data_list_[14];
    robot_data_list_custom[15] = robot_data_list_[15];
    robot_data_list_custom[16] = robot_data_list_[16];
    robot_data_list_custom[17] = robot_data_list_[17];
    robot_data_list_custom[18] = robot_data_list_[23];
    robot_data_list_custom[19] = robot_data_list_[22];
    robot_data_list_custom[20] = robot_data_list_[21];
    robot_data_list_custom[21] = robot_data_list_[20];
    robot_data_list_custom[22] = robot_data_list_[19];
    robot_data_list_custom[23] = robot_data_list_[18];
    robot_data_list_custom[24] = robot_data_list_[24];
    robot_data_list_custom[25] = robot_data_list_[25];
    robot_data_list_custom[26] = robot_data_list_[26];
    robot_data_list_custom[27] = robot_data_list_[27];
    robot_data_list_custom[28] = robot_data_list_[28];
    robot_data_list_custom[29] = robot_data_list_[29];

    // Overwrite robot data list with custom robot data list
    robot_data_list_ = robot_data_list_custom;
}

void GlobalPlanner::printPlanningOrder() {
    std::stringstream ss;
    ss << "New Planning Order: [";

    // Build the string with stage and robot ID information for each robot
    for (size_t i = 0; i < robot_data_list_.size(); ++i) {
        ss << "(stage " << robot_data_list_[i].stage 
           << ", robot " << robot_data_list_[i].robot_id << ")";
        
        if (i < robot_data_list_.size() - 1) {
            ss << ", ";
        } else {
            ss << "].";
        }
    }

    // Print the planning order
    ROS_INFO_STREAM(ss.str());
}

void GlobalPlanner::publishPath(const nav_msgs::Path gui_path, int robot_id) {
    // Publish path on the corresponding publisher
    path_publishers_[robot_id - 1].publish(gui_path);
    ROS_INFO("Published path with %ld waypoints on topic: /robot%d/global_path", gui_path.poses.size(), robot_id);
}

void GlobalPlanner::publishCablePath(const std::vector<std::pair<int, int>>& path, int robot_id) {    
    // Initialize path variable for publishing
    nav_msgs::Path gui_cable_path;
    gui_cable_path.header.frame_id = "map";
    gui_cable_path.header.stamp = ros::Time::now();

    // Convert each point in the path to world coordinates and add it to the path
    for (const auto& p : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        std::pair<double, double> world_coords = mapToWorld(p.first, p.second);
        pose.pose.position.x = world_coords.first;
        pose.pose.position.y = world_coords.second;
        pose.pose.orientation.w = 1.0;

        gui_cable_path.poses.push_back(pose);
    }
    
    // Publish cable path on the corresponding publisher
    cable_path_publishers_[robot_id - 1].publish(gui_cable_path);
    ROS_INFO("Published cable path with %ld waypoints on topic: /robot%d/cable_path", path.size(), robot_id);
    
    // Store the cable path in the robot data list
    robot_data_list_[pair_nr - 1].gui_cable_path = gui_cable_path;
}

void GlobalPlanner::publishScoringMetric() {
    // Update the scoring metric variable
    scoring_metric_.robot_quantity = robot_quantity_;
    scoring_metric_.pair_quantity = pair_quantity_;
    scoring_metric_.paths_planned = paths_planned_;
    scoring_metric_.ripped_up_nr = ripped_up_nr_;
    scoring_metric_.completion_rate = completion_rate_;
    scoring_metric_.pairs_failed_nr = pairs_failed_nr_;
    scoring_metric_.failed_routed_nr = failed_routed_nr_;
    scoring_metric_.rerouted_nr = rerouted_nr_;
    scoring_metric_.routability = routability_;
    scoring_metric_.reroutability = reroutability_;
    scoring_metric_.computing_time = computing_time_;
    scoring_metric_.cable_length = cable_length_;
    scoring_metric_.cable_length_ratio = cable_length_ratio_;
    scoring_metric_.path_length = path_length_;
    scoring_metric_.path_length_ratio = path_length_ratio_;

    // Publish scoring metric
    scoring_metric_pub_.publish(scoring_metric_);
}

void GlobalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // Initialize the map and costmap
    if (map_.data.empty()) {  // Receive the map only once
        map_ = *msg;
        costmap_ = map_;  // Initialize costmap_ as a copy of map_
        ROS_INFO("Map received and costmap initialized.");
    }
    
    // Store a copy of the cost map as static cost map
    costmap_static_ = costmap_;

    // Store costmap parameters
    costmap_width_ = costmap_.info.width;
    costmap_height_ = costmap_.info.height;
    costmap_resolution_ = costmap_.info.resolution;
    origin_x_ = costmap_.info.origin.position.x;
    origin_y_ = costmap_.info.origin.position.y;

    // Set inflation radii of robot and cable
    inflation_radius_robot_ = 1.5;
    inflation_radius_cable_ = 0.8;

    // Scale inflation radii to number of costmap cells
    cells_radius_robot_ = static_cast<int>(std::ceil(inflation_radius_robot_ / costmap_resolution_));
    cells_radius_cable_ = static_cast<int>(std::ceil(inflation_radius_cable_ / costmap_resolution_));

    // Register all cells with a value above 90 as obstacles
    for (int i = 0; i < costmap_height_; ++i) {
        for (int j = 0; j < costmap_width_; ++j) {
            int index = i * costmap_width_ + j;
            if (costmap_.data[index] > 90) {
                costmap_obstacles_.push_back(std::make_pair(i, j));
            }
        }
    }

    // Mark inflation zone for static obstacles
    ROS_INFO("Marking inflation zone of static obstacles");
    for (size_t i = 0; i < costmap_obstacles_.size(); ++i) {
        markInflationZone(costmap_obstacles_[i].first, costmap_obstacles_[i].second, cells_radius_cable_);
    }

    // Publish updated costmaps
    costmap_pub_.publish(costmap_);
    costmap_static_pub_.publish(costmap_static_);

    // Notify that the planner is ready
    if (!planner_ready_) {
        planner_ready_ = true;
        std_msgs::Bool ready_msg;
        ready_msg.data = true;
        ready_pub_.publish(ready_msg);  // Publish that the planner is ready
        ROS_INFO("GlobalPlanner is ready.");
    }
}

void GlobalPlanner::markInflationZone(int cell_x, int cell_y, int radius) {
    // Iterate over a square grid of radius around the given cell
    for (int i = -radius; i <= radius; ++i) {
        for (int j = -radius; j <= radius; ++j) {
            int new_x = cell_x + i;
            int new_y = cell_y + j;

            // Check if the point is within the inflation radius and bounds of the costmap
            if (std::hypot(i * costmap_resolution_, j * costmap_resolution_) <= inflation_radius_cable_
                && new_x >= 0 && new_x < costmap_width_ && new_y >= 0 && new_y < costmap_height_) {
                
                // Calculate the index for the costmap and mark it as occupied (value 100)
                int index = new_x * costmap_width_ + new_y;
                
                // Update costmap value to 100 if it's not already at or above 100
                if (costmap_static_.data[index] < 100) {
                    costmap_static_.data[index] = 100;
                }
            }
        }
    }
}

void GlobalPlanner::pairsCallback(const multi_robot_planner::StartGoalPairList::ConstPtr& msg) {
    // Wait for planner to be ready before processing the message
    ros::Rate loop_rate(1);
    while (ros::ok() && !planner_ready_) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // Store the received start-goal pairs
    start_goal_pairs_ = msg->pairs;
    processing_done_ = false;
    ROS_INFO("Received %ld start-goal pairs.", start_goal_pairs_.size());
    
    // Resize robot data list and initialize fields for each robot
    robot_data_list_.resize(start_goal_pairs_.size());
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_data_list_[i].robot_id = start_goal_pairs_[i].robot_id;
        robot_data_list_[i].stage = start_goal_pairs_[i].stage;
        robot_data_list_[i].start_goal_pair = start_goal_pairs_[i];
        robot_data_list_[i].ripped_up = false;
    }
    ROS_INFO("Robot data list created");

    // Determine the maximum robot ID and set up costmap variables
    auto max_it = std::max_element(robot_data_list_.begin(), robot_data_list_.end(),
        [](const RobotData& a, const RobotData& b) {
            return a.robot_id < b.robot_id;
        }
    );
    robot_quantity_ = (max_it != robot_data_list_.end()) ? max_it->robot_id : 0;
    costmaps_.resize(robot_quantity_);
    costmaps_path_.resize(robot_quantity_);
    costmap_pubs_.resize(robot_quantity_);
    costmap_path_pubs_.resize(robot_quantity_);
    path_publishers_.resize(robot_quantity_);
    cable_path_publishers_.resize(robot_quantity_);

    pair_quantity_ = robot_data_list_.size();

    // Advertise topics for each robot
    for (int i = 0; i < robot_quantity_; ++i) {
        costmaps_[i] = costmap_;
        costmaps_path_[i] = costmap_;

        std::string topic_name_costmap = "/global_planner_costmap_" + std::to_string(i + 1);
        std::string topic_name_costmap_path = "/global_planner_costmap_path_" + std::to_string(i + 1);
        std::string topic_name_path = "/robot" + std::to_string(i + 1) + "/global_path";
        std::string topic_name_cable_path = "/robot" + std::to_string(i + 1) + "/cable_path";

        costmap_pubs_[i] = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name_costmap, 1);
        costmap_path_pubs_[i] = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name_costmap_path, 1);
        path_publishers_[i] = nh_.advertise<nav_msgs::Path>(topic_name_path, 1);
        cable_path_publishers_[i] = nh_.advertise<nav_msgs::Path>(topic_name_cable_path, 1);
    }

    // Set pairs_ready_ flag after processing
    if (!pairs_ready_) {
        pairs_ready_ = true;
    }
    
    // Process the pairs
    processPairs();
}

void GlobalPlanner::updatePoint(int x, int y, int robot_id, int cost_value) {
    // Check if the point is within costmap bounds
    if (x >= 0 && x < costmap_width_ && y >= 0 && y < costmap_height_) {
        int index = y * costmap_width_ + x; // Calculate index in the costmap
        costmaps_[robot_id - 1].data[index] = cost_value;  // Update the cost value at the point
        ROS_DEBUG("Updated costmap at (%d, %d) with index %d", x, y, index);
    } else {
        ROS_WARN("Point (%d, %d) is out of costmap bounds", x, y);
    }
}

void GlobalPlanner::updatePointPath(int x, int y, int robot_id, int cost_value) {
    // Check if the point is within costmap bounds
    if (x >= 0 && x < costmap_width_ && y >= 0 && y < costmap_height_) {
        int index = y * costmap_width_ + x;// Calculate index in the costmap
        costmaps_path_[robot_id - 1].data[index] = cost_value;  // Update the cost value at the point
        ROS_DEBUG("Updated costmap at (%d, %d) with index %d", x, y, index);
    } else {
        ROS_WARN("Point (%d, %d) is out of costmap bounds", x, y);
    }
}

void GlobalPlanner::updateCostmapWithPath(int pair_index) {  
    // Get the path from the robot data list
    std::vector<std::pair<int, int>> path = robot_data_list_[pair_index].path;
    
    // Lambda function for conversion of polar to cartesian corrdinates
    auto polarToCartesian = [](double radius, double angle, double cx, double cy) -> std::pair<double, double> {
        double x = cx + radius * std::cos(angle);  // Berechne die X-Koordinate relativ zum Zentrum
        double y = cy + radius * std::sin(angle);  // Berechne die Y-Koordinate relativ zum Zentrum
        return std::make_pair(x, y);
    };
    
    std::vector<std::pair<int, int>> cable_circle;
    
    // Function to create a circular cable path
    auto createCableCircle = [&] (const std::pair<int, int>& center_point,
                                  const std::pair<int, int>& circle_point_start,
                                  const std::pair<int, int>& circle_point_end,
                                  double radius) -> std::vector<std::pair<int, int>> {

        // Transform map to world coordinates
        std::pair<double, double> world_center_point = mapToWorld(center_point.first, center_point.second);
        std::pair<double, double> world_circle_start = mapToWorld(circle_point_start.first, circle_point_start.second);
        std::pair<double, double> world_circle_end = mapToWorld(circle_point_end.first, circle_point_end.second);
    
        // Compute the angles of the points on the circle relative to the center in world coords
        double angle1 = std::atan2(world_circle_start.second - world_center_point.second, world_circle_start.first - world_center_point.first);
        double angle2 = std::atan2(world_circle_end.second - world_center_point.second, world_circle_end.first - world_center_point.first);

        // Make sure that the angles are within between 0 and 2π
        if (angle1 < 0) angle1 += 2 * M_PI;
        if (angle2 < 0) angle2 += 2 * M_PI;

        // Compute the shorter angle between the two circle points relative to the center
        double delta_angle = angle2 - angle1;
        if (delta_angle < 0) delta_angle += 2 * M_PI;
        if (delta_angle >= M_PI) delta_angle = -(2 * M_PI - delta_angle);

        // Number of steps on the circle
        int steps = static_cast<int>(std::abs(delta_angle) / (costmap_resolution_ / radius));

        // Compute the points on the circle
        if (steps >= 1) {
            for (int i = 0; i <= steps; ++i) {
                double current_angle = angle1 + i * delta_angle / steps;
        
                std::pair<double, double> world_point = polarToCartesian(radius, current_angle, world_center_point.first, world_center_point.second);
        
                std::pair<int, int> map_point = worldToMap(world_point.first, world_point.second);
        
                cable_circle.push_back(map_point);
            }
        }
    
        return cable_circle;
    };

    // Clear previous path costmap points
    path_costmap_points_set_.clear();
    
    // Initialize the cable path
    std::vector<std::pair<int, int>> cable_path;
    cable_path.emplace_back(robot_data_list_[pair_index].cable_start);

    // Create cable path with scaling based on points along the robot's path
    for (size_t i = 3; i < path.size(); ++i) {
        std::pair<int, int> point1;
        std::pair<int, int> point2;
        std::pair<int, int> point;
        
        // Determine adjacent points for calculating cable segments
        if (path.size() - i < 4) {
            point = path[i];
            point1 = path[i];
            point2 = path[i-7];
        } else if (i < 5 || path.size() - i < 6) {
            point = path[i];
            point1 = path[i+3];
            point2 = path[i-3];
        } else if (i < 10 || path.size() - i < 11) {
            point = path[i];
            point1 = path[i+5];
            point2 = path[i-5];
        } else {
            point = path[i];
            point1 = path[i+6];
            point2 = path[i-10];
        }
        
        int point1x = point1.first;
        int point1y = point1.second;
        int point2x = point2.first;
        int point2y = point2.second;
        int pointx = point.first;
        int pointy = point.second;
        
        // Scale and adjust the cable position based on the path
        double distance = std::sqrt(std::pow(point2x - point1x, 2) + std::pow(point2y - point1y, 2)) * costmap_resolution_;
        double scale_factor = 0.9 / distance;
        int cablex = std::round(pointx + (point2x - point1x) * scale_factor);
        int cabley = std::round(pointy + (point2y - point1y) * scale_factor);
        cable_path.emplace_back(cablex, cabley);
    }

    // Create cable circle around start point and append to cable path
    createCableCircle(path[0], robot_data_list_[pair_index].cable_start, cable_path[1], 0.9);   
    if (!cable_circle.empty()) {
        cable_path.insert(cable_path.begin() + 1, cable_circle.begin(), cable_circle.end());
    }

    cable_circle.clear();

    // Create cable circle around end point and append to cable path
    createCableCircle(path[path.size()-1], cable_path[cable_path.size()-1], robot_data_list_[pair_index].cable_goal, 0.9);
    if (!cable_circle.empty()) {
        cable_path.insert(cable_path.end(), cable_circle.begin(), cable_circle.end());
    }

    // Store the cable path in the robot data list
    robot_data_list_[pair_index].cable_path = cable_path;
    
    // Update costmap with the cable path
    for (size_t i = 0; i < cable_path.size(); ++i) {
        int mx = cable_path[i].first;
        int my = cable_path[i].second;
        
        for (int dx = -cells_radius_cable_; dx <= cells_radius_cable_; ++dx) {
            for (int dy = -cells_radius_cable_; dy <= cells_radius_cable_; ++dy) {
                int nx = mx + dx;
                int ny = my + dy;
                // Check if the point is within the inflation radius
                if (std::hypot(dx * costmap_resolution_, dy * costmap_resolution_) <= inflation_radius_cable_) {
                    // Check if the point (x, y) is already in path_costmap_points_set_
                    if (path_costmap_points_set_.find(std::make_pair(nx, ny)) == path_costmap_points_set_.end()) {
                        path_costmap_points_set_.emplace(nx, ny);

                        if (allow_crossing_ == false) {
                            updatePointPath(nx, ny, robot_data_list_[pair_index].robot_id, 80);
                        } else {
                            updatePointPath(nx, ny, robot_data_list_[pair_index].robot_id, 99);
                        }
                    }
                }
            }
        }
    }
    
    // Publish the updated costmap for the robot
    ROS_INFO("Publishing updated costmap on /global_planner_costmap.");
    costmap_path_pubs_[robot_data_list_[pair_index].robot_id - 1].publish(costmaps_path_[robot_data_list_[pair_index].robot_id - 1]);  // Publish the updated costmap
}

void GlobalPlanner::measureCableLength(int path_index) {
    ROS_INFO("Measuring cable length");

    // Calculate the total length of the robot's path
    std::vector<std::pair<int, int>> path = robot_data_list_[path_index].path;
    double path_length = 0.0;
    double path_segment_length = 0.0;
    if (path.size() > 2) {
        for (size_t i = 0; i < path.size() - 2; i += 2) {
            int point1x = path[i].first;
            int point1y = path[i].second;
            int point2x = path[i+2].first;
            int point2y = path[i+2].second;
            
            path_segment_length = std::sqrt(std::pow(point2x - point1x, 2) + std::pow(point2y - point1y, 2)) * costmap_resolution_;
            path_length += path_segment_length;
        }
    }

    // Store the path length in the robot data list
    robot_data_list_[pair_nr - 1].path_length = path_length;
    
    // Calculate the total length of the robot's cable path
    std::vector<std::pair<int, int>> cable_path = robot_data_list_[path_index].cable_path;
    double cable_length = 0.0;
    double cable_segment_length = 0.0;
    if (cable_path.size() > 3) {
        for (size_t i = 0; i < cable_path.size() - 3; i += 3) {
            int point1x = cable_path[i].first;
            int point1y = cable_path[i].second;
            int point2x = cable_path[i+3].first;
            int point2y = cable_path[i+3].second;
            
            cable_segment_length = std::sqrt(std::pow(point2x - point1x, 2) + std::pow(point2y - point1y, 2)) * costmap_resolution_;
            cable_length += cable_segment_length;
        }
    }

    // Store the cable path length in the robot data list
    robot_data_list_[pair_nr - 1].cable_length = cable_length;
    
    ROS_INFO("Cable length of pair %d (stage %d, robot %d): %f meters.", pair_nr, robot_data_list_[pair_nr-1].stage, robot_data_list_[pair_nr-1].robot_id, cable_length);
}

void GlobalPlanner::updateCostmapsWithStartGoal () {
    // Mark areas around start and goal positions as obstacles
    for (size_t i = 0; i < robot_data_list_.size(); ++i) {
        // Get start and goal position
        const auto& start = robot_data_list_[i].start_goal_pair.start;
        const auto& goal = robot_data_list_[i].start_goal_pair.goal;
        // Transform start and goal position to map coordinates
        auto start_map = worldToMap(start.pose.position.x, start.pose.position.y);
        auto goal_map = worldToMap(goal.pose.position.x, goal.pose.position.y);

        // Inflate areas around start and goal position and mark them as obstacle
        for (int dx = -cells_radius_robot_; dx <= cells_radius_robot_; ++dx) {
            for (int dy = -cells_radius_robot_; dy <= cells_radius_robot_; ++dy) {
                int nx_start = start_map.first + dx;
                int ny_start = start_map.second + dy;
                int nx_goal = goal_map.first + dx;
                int ny_goal = goal_map.second + dy;
                
                // Check if the point is within the inflation radius
                if (std::hypot(dx * costmap_resolution_, dy * costmap_resolution_) <= inflation_radius_robot_) {
                    updatePoint(nx_start, ny_start, robot_data_list_[i].robot_id, 100);
                    robot_data_list_[i].start_points.push_back(std::make_pair(nx_start, ny_start));
                    updatePoint(nx_goal, ny_goal, robot_data_list_[i].robot_id, 100);
                    robot_data_list_[i].goal_points.push_back(std::make_pair(nx_goal, ny_goal));
                }
            }
        }
    }

    // Mark cable points at start and goal positions as obstacles
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        // Get start and goal position
        const auto& start = robot_data_list_[i].start_goal_pair.start;
        const auto& goal = robot_data_list_[i].start_goal_pair.goal;
        // Get yaw of start and goal pose
        double start_yaw = getYawFromPose(start);
        double goal_yaw = getYawFromPose(goal);
        // Calculate cable points at start and goal position
        double cable_start_x = start.pose.position.x - 0.9 * cos(start_yaw);
        double cable_start_y = start.pose.position.y - 0.9 * sin(start_yaw);
        double cable_goal_x = goal.pose.position.x - 0.9 * cos(goal_yaw);
        double cable_goal_y = goal.pose.position.y - 0.9 * sin(goal_yaw);
        // Transform cable points to map coordinates
        auto cable_start_map = worldToMap(cable_start_x, cable_start_y);
        auto cable_goal_map = worldToMap(cable_goal_x, cable_goal_y);
        
        // Store cable points at start and goal in robot data list
        robot_data_list_[i].cable_goal = cable_goal_map;
        robot_data_list_[i].cable_start = cable_start_map;
        
        // Inflate areas around cable points at start and goal and mark as obstacles
        for (int dx = -cells_radius_cable_; dx <= cells_radius_cable_; ++dx) {
            for (int dy = -cells_radius_cable_; dy <= cells_radius_cable_; ++dy) {
                int nx_cable_start = cable_start_map.first + dx;
                int ny_cable_start = cable_start_map.second + dy;
                int nx_cable_goal = cable_goal_map.first + dx;
                int ny_cable_goal = cable_goal_map.second + dy;
            
                // Check if the point is within the inflation radius
                if (std::hypot(dx * costmap_resolution_, dy * costmap_resolution_) <= inflation_radius_cable_) {
                    // Update points inside the inflation zone of the cable start point and store them in the robot data list
                    updatePoint(nx_cable_start, ny_cable_start, robot_data_list_[i].robot_id, 100);
                    robot_data_list_[i].cable_start_points.emplace_back(nx_cable_start, ny_cable_start);
                    
                    // Update points inside the inflation zone of the cable goal point and store them in the robot data list
                    updatePoint(nx_cable_goal, ny_cable_goal, robot_data_list_[i].robot_id, 100);
                    robot_data_list_[i].cable_goal_points.emplace_back(nx_cable_goal, ny_cable_goal);
                }
            }
        }
    }

    // Publish the updated costmaps
    ROS_INFO("Publishing updated costmaps on /global_planner_costmap.");
    for (int i = 0; i < robot_quantity_; ++i) {
        costmap_pubs_[i].publish(costmaps_[i]);
    }
}

void GlobalPlanner::ripUpPath(int path_index) {
    ROS_WARN("Ripping up path %d (stage %d, robot %d)", path_index + 1, robot_data_list_[path_index].stage, robot_data_list_[path_index].robot_id);
    
    // Check if a point is part of another robot's path
     auto isPointInPath = [&](int x, int y) -> bool {
        for (size_t i = 0; i < robot_data_list_.size(); ++i) {
            if (i == path_index) {
                continue;
            }

            if (robot_data_list_[i].robot_id == robot_data_list_[path_index].robot_id) {
                if (robot_data_list_[i].path_costmap_points_set.find(std::make_pair(x, y)) != robot_data_list_[i].path_costmap_points_set.end()) {
                    return true;
                }
            }
        }
        return false;
    };

    // Check if a point is part of the robot's cable start points
    auto isPointStartCable = [&](int x, int y) -> bool {
        for (int i = 0; i < robot_data_list_.size(); ++i) {
            if (robot_data_list_[i].robot_id == robot_data_list_[pair_nr - 1].robot_id) {
                if (std::find(robot_data_list_[i].cable_start_points.begin(),
                    robot_data_list_[i].cable_start_points.end(),
                    std::make_pair(x, y)) != robot_data_list_[i].cable_start_points.end()) {
                        return true;
                }
            }
        }
        return false;
    };

    // Update costmap points to free space, excluding those involved in other paths
    for (const auto& point : robot_data_list_[path_index].path_costmap_points_set) {
        int mx = point.first;
        int my = point.second;
        if (!isPointInPath(mx, my)) {
            updatePointPath(mx, my, robot_data_list_[path_index].robot_id, 0); // Set point to free space
        } else {
            ROS_DEBUG("Skipped setting (%d, %d) to free space as it's part of the path", mx, my);
        }
    }
    
    // Publish updated costmap
    ROS_INFO("Publishing updated costmap on /global_planner_costmap.");
    costmap_path_pubs_[robot_data_list_[path_index].robot_id - 1].publish(costmaps_path_[robot_data_list_[path_index].robot_id - 1]);  // Publish the updated costmap
    
    // Clear path-related data and publish empty paths
    robot_data_list_[path_index].path.clear();
    robot_data_list_[path_index].gui_path.poses.clear();
    robot_data_list_[path_index].cable_path.clear();
    robot_data_list_[path_index].gui_cable_path.poses.clear();
    robot_data_list_[path_index].path_costmap_points.clear();
    robot_data_list_[path_index].path_costmap_points_set.clear();
    robot_data_list_[path_index].cable_length = 0.0;

    publishPath(robot_data_list_[path_index].gui_path, robot_data_list_[path_index].robot_id);
    publishCablePath(robot_data_list_[path_index].cable_path, robot_data_list_[path_index].robot_id);
    
    // Update number of planned and ripped up paths
    --paths_planned_;
    ++ripped_up_nr_;
    robot_data_list_[path_index].ripped_up = true;
}


void GlobalPlanner::ripUpRandom() {
    ROS_INFO("Ripping up random by cable.");

    // Vectors to categorize robot data
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    std::vector<int> randomizer;
    
    // Fill randomizer with robot indexes
    for (int i = 0; i < pair_nr - 1; ++i) {
        randomizer.push_back(i);
    }

    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Determine the number of paths to remove
    int n_rip;
    if (paths_planned_ == 1) {
        n_rip = 1;
    } else {
        n_rip = paths_planned_ / 2;
    }
    
    // Select and sort random paths to remove
    std::mt19937 rng(std::time(nullptr));
    std::shuffle(randomizer.begin(), randomizer.end(), rng);
    std::vector<int> paths_ripped(randomizer.begin(), randomizer.begin() + n_rip);
    std::sort(paths_ripped.begin(), paths_ripped.end());
    std::vector<int> paths_not_ripped(randomizer.begin() + n_rip, randomizer.end());
    std::sort(paths_not_ripped.begin(), paths_not_ripped.end());
    
    // Remove random paths and store their data
    for (size_t i = 0; i < paths_ripped.size(); ++i) {
        ripUpPath(paths_ripped[i]);
        pairs_routed_ripped.push_back(robot_data_list_[paths_ripped[i]]);
    }
    
    // Store data for non-ripped paths
    for (size_t i = 0; i < paths_not_ripped.size(); ++i) {
        pairs_routed_not_ripped.push_back(robot_data_list_[paths_not_ripped[i]]);
    }
    
    // Reorganize robot data list
    ROS_INFO("Resort robot data list");
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Update robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Reduce the pair number counter
    pair_nr -= n_rip;
    
    // Print the start and goal positions of all remaining paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::ripUpByLength() {
    ROS_INFO("Ripping up paths by cable length.");

    // Vectors to categorize robot data
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    
    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign all remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Fill vector with cable lengths
    robot_cable_lengths_sorted_.resize(paths_planned_);
    for (int i = 0; i < pair_nr - 1; ++i) {
        robot_cable_lengths_sorted_[i].index = i;
        robot_cable_lengths_sorted_[i].length = robot_data_list_[i].cable_length;
        robot_cable_lengths_sorted_[i].robot_id = robot_data_list_[i].robot_id;
    }
    
    // Sort paths by cable length in descending order (longest first)
    std::sort(robot_cable_lengths_sorted_.begin(), robot_cable_lengths_sorted_.end(), 
              [](const RobotCableLengthSorted& a, const RobotCableLengthSorted& b) {
                  return a.length > b.length;
              });
    
    // Determine the number of paths to remove
    int n_rip;
    if (paths_planned_ < 4) {
        n_rip = 1;
    } else {
        n_rip = paths_planned_ / 4;
    }
    
    // Remove the longest paths and store their indices
    std::vector<int> pairs_routed_ripped_indexes;
    for (int i = 0; i < n_rip; ++ i) {
        ripUpPath(robot_cable_lengths_sorted_[i].index);
        pairs_routed_ripped_indexes.push_back(robot_cable_lengths_sorted_[i].index);
    }
    // Sort the indices of removed paths
    std::sort(pairs_routed_ripped_indexes.begin(), pairs_routed_ripped_indexes.end());

    // Store data for ripped paths
    for (size_t i = 0; i < pairs_routed_ripped_indexes.size(); ++i) {
        pairs_routed_ripped.push_back(robot_data_list_[pairs_routed_ripped_indexes[i]]);
    }
    
    // Identify paths that were not removed
    std::vector<int> pairs_routed_not_ripped_indexes;
    if (!paths_planned_ == 0) {
        for (int i = n_rip; i < pair_nr - 1; ++i) {
            pairs_routed_not_ripped_indexes.push_back(robot_cable_lengths_sorted_[i].index);
        }
    }

    // Sort the indices of non-ripped paths
    std::sort(pairs_routed_not_ripped_indexes.begin(), pairs_routed_not_ripped_indexes.end());
    
    // Store data for non-ripped paths
    for (size_t i = 0; i < pairs_routed_not_ripped_indexes.size(); ++i) {
        pairs_routed_not_ripped.push_back(robot_data_list_[pairs_routed_not_ripped_indexes[i]]);
    }
    
    // Reorganize robot data list
    ROS_INFO("Resort robot data list");
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Update robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Reduce the pair number counter
    pair_nr -= n_rip;
    
    // Print the start and goal positions of all remaining paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::ripUpByLengthRatio() {
    ROS_INFO("Ripping up paths by ratio of path length to distance of start and goal.");

    // Vectors to store robot data categorized into different groups
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    
    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign all remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Fill vector with length ratios (path length / direct distance)
    robot_length_ratios_sorted_.resize(paths_planned_);
    for (int i = 0; i < pair_nr - 1; ++i) {
        robot_length_ratios_sorted_[i].index = i;
        robot_length_ratios_sorted_[i].ratio = robot_data_list_[i].path_length / robot_data_list_[i].distance;
    }
    
    // Sort paths based on length ratio in descending order (longest first)
    std::sort(robot_length_ratios_sorted_.begin(), robot_length_ratios_sorted_.end(), 
              [](const LengthRatioSorted& a, const LengthRatioSorted& b) {
                  return a.ratio > b.ratio;
              });
    
    // Determine how many paths to remove
    int n_rip;
    if (paths_planned_ < 4) {
        n_rip = 1;
    } else {
        n_rip = paths_planned_ / 4;
    }

    // Rip up the longest paths and and store their indices
    std::vector<int> pairs_routed_ripped_indexes;
    for (int i = 0; i < n_rip; ++ i) {
        ripUpPath(robot_length_ratios_sorted_[i].index);
        pairs_routed_ripped_indexes.push_back(robot_length_ratios_sorted_[i].index);
    }
    // Sort indices of removed paths
    std::sort(pairs_routed_ripped_indexes.begin(), pairs_routed_ripped_indexes.end());

    // Store data for ripped paths
    for (size_t i = 0; i < pairs_routed_ripped_indexes.size(); ++i) {
        pairs_routed_ripped.push_back(robot_data_list_[pairs_routed_ripped_indexes[i]]);
    }
    
    // Identify paths that were not removed
    std::vector<int> pairs_routed_not_ripped_indexes;
    if (!paths_planned_ == 0) {
        for (int i = n_rip; i < pair_nr - 1; ++i) {
            pairs_routed_not_ripped_indexes.push_back(robot_length_ratios_sorted_[i].index);
        }
    }
    
    // Sort indices of non-ripped paths
    std::sort(pairs_routed_not_ripped_indexes.begin(), pairs_routed_not_ripped_indexes.end());
    
    // Store data for non-ripped paths
    for (size_t i = 0; i < pairs_routed_not_ripped_indexes.size(); ++i) {
        pairs_routed_not_ripped.push_back(robot_data_list_[pairs_routed_not_ripped_indexes[i]]);
    }
    
    // Reorganize robot data list with new order
    ROS_INFO("Resort robot data list");
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Update robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Reduce the pair number counter
    pair_nr -= n_rip;
    
    // Print the start and goal positions of all remaining paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::ripUpClosest() {
    ROS_INFO("Ripping up paths by closest distance to isolated nodes.");

    // Vectors to categorize robot data
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    
    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Lambda function to calculate Euclidean distance between two points
    auto distance = [&](int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);  // Euclidian distance
    };
    
    robot_path_distance_to_node_.clear();
    robot_path_distance_to_node_.resize(paths_planned_);
    
    // Calculate distances to isolated nodes
    for (int i = 0; i < pair_nr - 1; ++i) {
        // Get the start and goal positions of the failed path
        const auto start = robot_data_list_[pair_nr - 1].start_goal_pair.start;
        const auto start_map = worldToMap(start.pose.position.x, start.pose.position.y);
        int start_map_x = start_map.first;
        int start_map_y = start_map.second;
        
        const auto goal = robot_data_list_[pair_nr - 1].start_goal_pair.goal;
        const auto goal_map = worldToMap(goal.pose.position.x, goal.pose.position.y);
        int goal_map_x = goal_map.first;
        int goal_map_y = goal_map.second;
        
        // Get the path to be evaluated
        std::vector<std::pair<int, int>> path_dist_calc = robot_data_list_[i].path;
        
        // Initialize distance for this path
        robot_path_distance_to_node_[i].index = i;
        robot_path_distance_to_node_[i].distance = 1000;
        robot_path_distance_to_node_[i].robot_id = robot_data_list_[i].robot_id;
        
        // Calculate distance from path nodes to start and goal positions
        for (size_t j = 0; j < path_dist_calc.size(); ++j) {
            int path_x = path_dist_calc[j].first;
            int path_y = path_dist_calc[j].second;
            
            // Calculate distance to the start point
            double distance_start = distance(start_map_x, start_map_y, path_x, path_y);
            
            // Update the minimum distance if a smaller one is found
            if (distance_start < robot_path_distance_to_node_[i].distance) {
                robot_path_distance_to_node_[i].distance = distance_start;
            }
            
            // Calculate distance to the goal point
            double distance_goal = distance(goal_map_x, goal_map_y, path_x, path_y);
            
            // Update the minimum distance if a smaller one is found
            if (distance_goal < robot_path_distance_to_node_[i].distance) {
                robot_path_distance_to_node_[i].distance = distance_goal;
            }
        }
    }
    
    // Sort the paths based on their distance to the closest node in ascending order (closest first)
    std::sort(robot_path_distance_to_node_.begin(), robot_path_distance_to_node_.end(), 
              [](const PathDistanceToIsolatedNode& a, const PathDistanceToIsolatedNode& b) {
                  return a.distance < b.distance;
              }
    );
    
    // Set the number of paths to rip up
    int n_rip;
    if (paths_planned_ < 4) {
        n_rip = 1;
    } else {
        n_rip = paths_planned_ / 4;
    }
    
    // Rip up the closest paths and store their indices
    std::vector<int> pairs_routed_ripped_indexes;
    for (int i = 0; i < n_rip; ++ i) {
        ripUpPath(robot_path_distance_to_node_[i].index);
        pairs_routed_ripped_indexes.push_back(robot_path_distance_to_node_[i].index);
    }

    // Sort the indices of the ripped paths
    std::sort(pairs_routed_ripped_indexes.begin(), pairs_routed_ripped_indexes.end());
    
    // Store the data for the ripped paths
    for (size_t i = 0; i < pairs_routed_ripped_indexes.size(); ++i) {
        pairs_routed_ripped.push_back(robot_data_list_[pairs_routed_ripped_indexes[i]]);
    }
    
    // Assign indices to the vector of non-ripped paths
    std::vector<int> pairs_routed_not_ripped_indexes;
    if (!paths_planned_ == 0) {
        for (int i = n_rip; i < pair_nr - 1; ++i) {
            pairs_routed_not_ripped_indexes.push_back(robot_path_distance_to_node_[i].index);
        }
    }

    // Sort the indices of the non-ripped paths
    std::sort(pairs_routed_not_ripped_indexes.begin(), pairs_routed_not_ripped_indexes.end());
    
    // Store the data for the non-ripped paths
    for (size_t i = 0; i < pairs_routed_not_ripped_indexes.size(); ++i) {
        pairs_routed_not_ripped.push_back(robot_data_list_[pairs_routed_not_ripped_indexes[i]]);
    }
    
    // Reorganize the robot data list with the new order
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Resort the robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Decrease the pair number counter by the number of ripped paths
    pair_nr -= n_rip;
    
    // Print out the start and goal positions of the remaining paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::ripUpByCost() {
    // Vectors to categorize robot data
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    
    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Lambda function to calculate Euclidean distance between two points
    auto distance = [&](int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);  // Euclidian distance
    };
    
    // Clear and resize the vectors to hold calculated distances, cable lengths, and costs
    robot_path_distance_to_node_.clear();
    robot_path_distance_to_node_.resize(paths_planned_);
    robot_cable_lengths_sorted_.clear();
    robot_cable_lengths_sorted_.resize(paths_planned_);
    robot_path_cost_.clear();
    robot_path_cost_.resize(paths_planned_);
    
    // Calculate the path cost for each robot
    for (int i = 0; i < pair_nr - 1; ++i) {
        // Retrieve start and goal positions for comparison
        const auto start = robot_data_list_[pair_nr - 1].start_goal_pair.start;
        const auto start_map = worldToMap(start.pose.position.x, start.pose.position.y);
        int start_map_x = start_map.first;
        int start_map_y = start_map.second;
        
        const auto goal = robot_data_list_[pair_nr - 1].start_goal_pair.goal;
        const auto goal_map = worldToMap(goal.pose.position.x, goal.pose.position.y);
        int goal_map_x = goal_map.first;
        int goal_map_y = goal_map.second;
        
        // Get the path of the current robot
        std::vector<std::pair<int, int>> path_dist_calc = robot_data_list_[i].path;
        
        // Initialize the distance to a large number and set the robot_id
        robot_path_distance_to_node_[i].index = i;
        robot_path_distance_to_node_[i].distance = 1000;
        robot_path_distance_to_node_[i].robot_id = robot_data_list_[i].robot_id;
        
        // Loop through the path and calculate distances to the start and goal
        for (size_t j = 0; j < path_dist_calc.size(); ++j) {
            int path_x = path_dist_calc[j].first;
            int path_y = path_dist_calc[j].second;
            
            // Calculate distance to the start point
            double distance_start = distance(start_map_x, start_map_y, path_x, path_y);
            
            // Update the minimum distance if a smaller one is found
            if (distance_start < robot_path_distance_to_node_[i].distance) {
                robot_path_distance_to_node_[i].distance = distance_start;
            }
            
            // Calculate distance to the goal point
            double distance_goal = distance(goal_map_x, goal_map_y, path_x, path_y);
            
            // Update the minimum distance if a smaller one is found
            if (distance_goal < robot_path_distance_to_node_[i].distance) {
                robot_path_distance_to_node_[i].distance = distance_goal;
            }
        }

        // Store cable lengths and robot ids
        robot_cable_lengths_sorted_[i].index = i;
        robot_cable_lengths_sorted_[i].length = robot_data_list_[i].cable_length;
        robot_cable_lengths_sorted_[i].robot_id = robot_data_list_[i].robot_id;

        // Calculate the path cost: cost is a weighted sum of cable length and inverse distance
        robot_path_cost_[i].index = i;
        robot_path_cost_[i].cost = 1 * robot_cable_lengths_sorted_[i].length + 80 / (robot_path_distance_to_node_[i].distance * costmap_resolution_);
    }
    
    // Sort robot path costs in descending order (highest cost first)
    std::sort(robot_path_cost_.begin(), robot_path_cost_.end(), 
              [](const PathCost& a, const PathCost& b) {
                  return a.cost > b.cost;
              }
    );

    // Determine how many paths to rip up
    int n_rip;
    if (paths_planned_ < 4) {
        n_rip = 1;
    } else {
        n_rip = paths_planned_ / 4;
    }
    
    // Rip up the paths with the highest cost and store their indices
    std::vector<int> pairs_routed_ripped_indexes;
    for (int i = 0; i < n_rip; ++ i) {
        ripUpPath(robot_path_cost_[i].index);
        pairs_routed_ripped_indexes.push_back(robot_path_cost_[i].index);
    }

    // Sort the indices of the ripped paths
    std::sort(pairs_routed_ripped_indexes.begin(), pairs_routed_ripped_indexes.end());

    // Store robot data for the ripped paths
    for (size_t i = 0; i < pairs_routed_ripped_indexes.size(); ++i) {
        pairs_routed_ripped.push_back(robot_data_list_[pairs_routed_ripped_indexes[i]]);
    }
    
    // Store indices of non-ripped paths
    std::vector<int> pairs_routed_not_ripped_indexes;
    if (!paths_planned_ == 0) {
        for (int i = n_rip; i < pair_nr - 1; ++i) {
            pairs_routed_not_ripped_indexes.push_back(robot_path_cost_[i].index);
        }
    }

    // Sort indices of non-ripped paths
    std::sort(pairs_routed_not_ripped_indexes.begin(), pairs_routed_not_ripped_indexes.end());
    
    // Store robot data for the non-ripped paths
    for (size_t i = 0; i < pairs_routed_not_ripped_indexes.size(); ++i) {
        pairs_routed_not_ripped.push_back(robot_data_list_[pairs_routed_not_ripped_indexes[i]]);
    }
    
    // Reorganize the robot data list
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Update the robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Decrease the pair number counter by the number of paths ripped
    pair_nr -= n_rip;
    
    // Print out the start and goal information of all remaining paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::ripUpCrossedPaths() {
    ROS_INFO("Ripping up paths crossed by the new path.");

    // Vectors to categorize robot data
    std::vector<RobotData> pairs_routed_not_ripped;
    std::vector<RobotData> pairs_routed_ripped;
    RobotData pair_failed;
    std::vector<RobotData> pairs_not_routed;
    int n_rip;
    
    // Assign the failed robot path to pair_failed
    pair_failed = robot_data_list_[pair_nr - 1];

    // Assign remaining unrouted paths to pairs_not_routed
    for (int i = pair_nr; i < start_goal_pairs_.size(); ++i) {
        pairs_not_routed.push_back(robot_data_list_[i]);
    }
    
    // Lambda function to check if the path crosses another path's inflation zone (costmap)
    auto doesPathCross = [&](std::vector<std::pair<int, int>> path,
                             std::unordered_set<std::pair<int, int>, pair_hash> path_costmap_points_set) -> bool {
        for (const auto& path_point : path) {
            if (path_costmap_points_set.find(path_point) != path_costmap_points_set.end()) {
                return true;
            }
        }
        return false;
    };
    
    // Check if any paths are crossed by the failed path
    std::vector<int> paths_crossed;
    for (int i = 0; i < pair_nr - 1; ++i) {
        if (doesPathCross(robot_data_list_[pair_nr - 1].path, robot_data_list_[i].path_costmap_points_set) == true
            && robot_data_list_[i].robot_id != robot_data_list_[pair_nr - 1].robot_id) {
            paths_crossed.push_back(i);
            ripUpPath(i);
            pairs_routed_ripped.push_back(robot_data_list_[i]);
        }
        else {
            pairs_routed_not_ripped.push_back(robot_data_list_[i]);
        }
    }

    // Store the number of ripped paths
    n_rip = paths_crossed.size();

    // Add the failed path's information to the dominant_paths list
    dominant_paths_.push_back(std::make_tuple(n_rip, robot_data_list_[pair_nr - 1].stage, robot_data_list_[pair_nr - 1].robot_id));

    // Reorganize the robot data list
    robot_data_list_.clear();
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_not_ripped.begin(), pairs_routed_not_ripped.end());
    robot_data_list_.push_back(pair_failed);
    robot_data_list_.insert(robot_data_list_.end(), pairs_routed_ripped.begin(), pairs_routed_ripped.end());
    robot_data_list_.insert(robot_data_list_.end(), pairs_not_routed.begin(), pairs_not_routed.end());
    
    // Update the robot order list
    robot_order_.clear();
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        robot_order_.push_back(robot_data_list_[i].robot_id);
    }
    
    // Update the pair number counter after ripping paths
    pair_nr = paths_planned_ + 1;
    
    // Print out the start and goal information for all the paths
    ROS_INFO("Printing start_goal_pairs_ information:");
    for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
        ROS_INFO("Pair %ld: Start (%f, %f), Goal (%f, %f)",
            i+1,
            robot_data_list_[i].start_goal_pair.start.pose.position.x, robot_data_list_[i].start_goal_pair.start.pose.position.y,
            robot_data_list_[i].start_goal_pair.goal.pose.position.x, robot_data_list_[i].start_goal_pair.goal.pose.position.y);
    }
}

void GlobalPlanner::initializeActionClients(const std::vector<int>& robot_ids) {
    for (int robot_id : robot_ids) {
        // Construct the action name dynamically based on the robot's ID
        std::string action_name = "/mir" + std::to_string(robot_id) + "/move_base_flex/exe_path";
        
        // Create a new action client for each robot
        action_clients_[robot_id] = std::make_shared<actionlib::SimpleActionClient<mbf_msgs::ExePathAction>>(action_name, true);
    }
}

void GlobalPlanner::sendAllPathsToRobots() {
    ROS_INFO("Sorting robot data list by stage number.");

    // Sort the robot data list by stage number
    std::sort(robot_data_list_.begin(), robot_data_list_.end(), 
        [](const RobotData& a, const RobotData& b) {
            return a.stage < b.stage;
        }
    );

    ROS_INFO("Sending paths to robots in the following order:");
    printPlanningOrder();

    for (int i = 0; i < robot_data_list_.size(); ++i) {
        // Get the robot ID
        int robot_id = robot_data_list_[i].robot_id;

        // Send the path to the corresponding robot
        sendPathToRobot(robot_id, robot_data_list_[i].gui_path);

        // Wait until the robot has reached its goal
        action_clients_[robot_id]->waitForResult();  // Access via the shared_ptr
        if (action_clients_[robot_id]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot %d has reached its goal.", robot_id);
        } else {
            ROS_WARN("Robot %d did not reach its goal.", robot_id);
        }
    }
}

void GlobalPlanner::sendPathToRobot(int robot_id, const nav_msgs::Path& path) {
    // Create an ExePathGoal object to store the path and controller
    mbf_msgs::ExePathGoal goal;
    
    // Set the path and controller for the goal
    goal.path = path;
    goal.controller = "DWAPlannerROS";

    // Send the goal to the robot via the action client
    action_clients_[robot_id]->sendGoal(goal);

    ROS_INFO("Sent path to robot %d.", robot_id);
}
