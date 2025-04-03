#include <pure_voronoi_planner/planning_utils.h>

namespace plan_utils
{

std::vector<cv::Point2i> create_deltas(bool use_eight_neighbors)
{
    std::vector<cv::Point2i> deltas;
    deltas.push_back(cv::Point2i(-1, 0));  // go up
    deltas.push_back(cv::Point2i(0, -1));  // go left
    deltas.push_back(cv::Point2i(1, 0));   // go down
    deltas.push_back(cv::Point2i(0, 1));   // go right
    // use only 4 neighborhood to get 90 degree curves only, which are better for later interpolation
    if (use_eight_neighbors)
    {
        deltas.push_back(cv::Point2i(-1, -1));  // up and left
        deltas.push_back(cv::Point2i(-1, 1));   // up and right
        deltas.push_back(cv::Point2i(1, -1));   // down and left
        deltas.push_back(cv::Point2i(1, 1));    // down and right
    }
    return deltas;
}


void mapToWorld(const cv::Point2i& point_map, cv::Point2d& point_world, cv::Point2d map_origin, double resolution)
{
    double half_a_cell = 0.5;
    point_world.x = map_origin.x + (point_map.x + half_a_cell) * resolution;
    point_world.y = map_origin.y + (point_map.y + half_a_cell) * resolution;
}


bool worldToMap(const cv::Point2d& point_world, cv::Point2i& point_map, cv::Point2d map_origin, int map_size_x,
                int map_size_y, double resolution)
{
    if (point_world.x < map_origin.x || point_world.y < map_origin.y)
        return false;

    point_map.x = (int)((point_world.x - map_origin.x) / resolution);
    point_map.y = (int)((point_world.y - map_origin.y) / resolution);

    if (point_map.x < map_size_x && point_map.y < map_size_y)
        return true;

    return false;
}

void pathMapToWorld(const std::vector<cv::Point2i>& path_in, std::vector<cv::Point3d>& path_world_out,
                    std::shared_ptr<costmap_2d::Costmap2D> costmap)
{
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();

    for (int path_idx = 0; path_idx < path_in.size(); path_idx++)
    {
        cv::Point2i this_point = path_in.at(path_idx);

        cv::Point2d point_world;
        mapToWorld(this_point, point_world, map_origin, map_resolution);

        cv::Point3d this_pose;
        this_pose.x = point_world.x;
        this_pose.y = point_world.y;

        if (path_idx == path_in.size() - 1)
        {
            if (path_in.size() == 1)
            {
                // cant compute orientation from one point, assume 0 rotation
                this_pose.z = 0.0;
            }
            else
            {
                // Last point in path so orientation of last pose will be taken
                this_pose.z = path_world_out.back().z;
            }
        }
        else
        {
            // Some other points are behind, so orientation can be calculated
            cv::Point2i next_point = path_in.at(path_idx + 1);
            cv::Point2d next_point_world;
            mapToWorld(next_point, next_point_world, map_origin, map_resolution);
            cv::Point2d delta = next_point_world - point_world;
            double yaw_angle = std::atan2(delta.y, delta.x);
            this_pose.z = yaw_angle;
        }
        path_world_out.push_back(this_pose);
    }
}


void createPlanFromPath(const std::vector<cv::Point3d>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out,
                        std::string global_frame_id)
{
    plan_out.clear();
    for (auto point: path_in)
    {
        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.frame_id = global_frame_id;
        curr_pose.header.stamp = ros::Time::now();
        curr_pose.pose.position.x = point.x;
        curr_pose.pose.position.y = point.y;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, point.z);
        curr_pose.pose.orientation = tf2::toMsg(quat);
        plan_out.push_back(curr_pose);
    }
}


bool isPlanFree(std::shared_ptr<costmap_2d::Costmap2D> costmap, int free_cell_threshold,
                const std::vector<geometry_msgs::PoseStamped>& plan)
{
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();
    for (auto pose : plan)
    {
        cv::Point2i point_map;
        worldToMap(cv::Point2d(pose.pose.position.x, pose.pose.position.y), point_map, map_origin, map_size_x,
                   map_size_y, map_resolution);

        if (costmap->getCost(point_map.x, point_map.y) > free_cell_threshold)
        {
            ROS_WARN_STREAM("Plan is not free at " << point_map.x << ", " << point_map.y);
            return false;
        }
    }
    // ROS_WARN("Plan is free!");
    return true;
}

}

namespace astar_planning
{

/** @brief calculates heuristic cost between cells with euclidean distance
 *
 */
float calcHCost(cv::Point2i current_cell, cv::Point2i target_cell)
{
    double l2_dist = std::sqrt(std::pow(current_cell.x - target_cell.x, 2) + std::pow(current_cell.y - target_cell.y, 2));
    double l1_dist = abs(current_cell.x - target_cell.x) + abs(current_cell.y - target_cell.y);
    return l1_dist;
}

float calcGCost(float current_cell_g_cost, cv::Point2i current_cell, cv::Point2i target_cell)
{
    return current_cell_g_cost + calcHCost(current_cell, target_cell);
}

float calcFCost(float current_cell_g_score, cv::Point2i current_cell, cv::Point2i target_cell, cv::Point2i goal_cell)
{
    return calcGCost(current_cell_g_score, current_cell, target_cell) + calcHCost(target_cell, goal_cell);
}

bool findAStarPathOnImage(const cv::Mat& voronoi_map, std::vector<cv::Point2i>& out_path, cv::Point2i start,
                          cv::Point2i goal, bool use_eight_neighbors)
{
    ROS_DEBUG("finding A-Star path on voronoi");
    // TODO: make sure start and goal are valid points and free
    bool start_is_voronoi = voronoi_map.at<uchar>(start.x, start.y) == 255;
    if (!start_is_voronoi)
    {
        ROS_ERROR("Start is not a valid voronoi cell!");
        return false;
    }
    bool goal_is_voronoi = voronoi_map.at<uchar>(goal.x, goal.y) == 255;
    if (!goal_is_voronoi)
    {
        ROS_ERROR("Goal is not a valid voronoi cell!");
        return false;
    }
    // create deltas used for neighborhood check
    std::vector<cv::Point2i> deltas = plan_utils::create_deltas(use_eight_neighbors);
    // crete g_score array
    cv::Mat g_score(voronoi_map.size(), CV_32FC1, std::numeric_limits<float>::infinity());
    g_score.at<float>(start.x, start.y) = 0;
    // fill gscore array with AStar
    std::multiset<AStarCell, std::less<AStarCell>> array_open_cell_list;
    array_open_cell_list.insert({ start, calcHCost(start, goal) });
    ROS_DEBUG("Creating g score array");
    int loop_counter = 0;

    bool timeout = false;
    double max_runtime = 1.0;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (!array_open_cell_list.empty() && g_score.at<float>(goal.x, goal.y) == std::numeric_limits<float>::infinity() && !timeout)
    {
        loop_counter++;
        // Get cell with lowest f_score and remove it so it will not be visited again
        cv::Point2i current_cell = array_open_cell_list.begin()->pixel;
        array_open_cell_list.erase(array_open_cell_list.begin());
        float current_gscore = g_score.at<float>(current_cell.x, current_cell.y);
        // get all free neighbors
        for (auto delta : deltas)
        {
            cv::Point2i neighbor_cell = current_cell + delta;
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                continue;
            }
            if (g_score.at<float>(neighbor_cell.x, neighbor_cell.y) == std::numeric_limits<float>::infinity())
            {
                g_score.at<float>(neighbor_cell.x, neighbor_cell.y) = calcGCost(current_gscore, current_cell, neighbor_cell);
                array_open_cell_list.insert(
                    { neighbor_cell, calcFCost(current_gscore, current_cell, neighbor_cell, goal) });
            }
        }
        std::chrono::steady_clock::time_point end_loop_time = std::chrono::steady_clock::now();
        double total_time_in_planning = (std::chrono::duration_cast<std::chrono::microseconds>(end_loop_time - start_time).count()) / 1000000.0;
        if (total_time_in_planning > max_runtime)
        {
            timeout = true;
        }
    }
    if (timeout)
    {
        ROS_ERROR("AStar timed out, possibly no path possible");
        return false;
    }
    if (g_score.at<float>(goal.x, goal.y) == std::numeric_limits<float>::infinity())
    {
        ROS_ERROR("No connection on voronoi found");
        return false;
    }
    ROS_DEBUG_STREAM("Loops needed for gscores: " << loop_counter);
    // Construct path backwards from goal to start to get best path
    ROS_DEBUG("Trace back path");
    out_path.clear();
    out_path.push_back(goal);
    cv::Point2i current_cell = goal;
    // bool is_start_cell = (current_cell.x == start.x) && (current_cell.y == start.y);
    // bool is_not_start_cell = current_cell != start;
    while (!((current_cell.x == start.x) && (current_cell.y == start.y)))
    {
        cv::Point2i min_g_score_cell = current_cell;
        ROS_DEBUG_STREAM("Current cell: " << current_cell);
        for (auto delta : deltas)
        {
            cv::Point2i neighbor_cell = current_cell + delta;
            ROS_DEBUG_STREAM("Neighbor cell: " << neighbor_cell);
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                ROS_DEBUG("Skipped cos not within map");
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                ROS_DEBUG("Skipped cos not on voronoi");
                continue;
            }
            if (g_score.at<float>(min_g_score_cell.x, min_g_score_cell.y) > g_score.at<float>(neighbor_cell.x, neighbor_cell.y))
            {
                min_g_score_cell = neighbor_cell;
            }
        }
        out_path.push_back(min_g_score_cell);
        current_cell = min_g_score_cell;
    }
    ROS_DEBUG("reversing path");
    std::reverse(out_path.begin(), out_path.end());
    ROS_DEBUG("Done");
    return true;
}
} // astar_planning


namespace dijkstra_planning
{

int findDijkstraPathToVoronoi(const cv::Mat& obstacle_map, const cv::Mat& voronoi_map,
                               cv::Point2i& nearest_point_on_voronoi, cv::Point2i start, cv::Point2i goal,
                               bool use_eight_neighbors)
{
    std::vector<cv::Point2i> deltas = plan_utils::create_deltas(use_eight_neighbors);
    cv::Mat visited(voronoi_map.size(), CV_8UC1, cv::Scalar(0));
    visited.at<uchar>(start.x, start.y) = 255;
    float g = 0;
    int movement_cost = 1;  // doesnt matter in which direction or angle, one cell movement costs 1

    // vector of open cells (for possible expansion) nodes
    std::vector<std::tuple<float, int, int>> open_cells;
    open_cells.push_back(std::make_tuple(g, start.x, start.y));

    // path found flag
    bool found_goal = false;
    // no solution could be found flag
    bool resign = false;

    while (!found_goal && !resign)
    {
        if (open_cells.size() == 0)
        {
            return PlanningStatus::Failed;
        }
        // sort open by cost
        std::sort(open_cells.begin(), open_cells.end());
        std::reverse(open_cells.begin(), open_cells.end());
        // get node with lowest cost
        std::tuple<float, int, int> next = open_cells[open_cells.size() - 1];
        open_cells.pop_back();
        g = std::get<0>(next);
        int curr_cell_x = std::get<1>(next);
        int curr_cell_y = std::get<2>(next);

        // check, whether the solution is found (we are at the goal)
        // we stop, when get path to any voronoi cell
        if (voronoi_map.at<uchar>(curr_cell_x, curr_cell_y) == VORONOI_VALUE)
        {
            nearest_point_on_voronoi.x = curr_cell_x;
            nearest_point_on_voronoi.y = curr_cell_y;
            // cv::line(visited, cv::Point2i(curr_cell_y, curr_cell_x), cv::Point2i(start.y, start.x), cv::Scalar(127));
            // cv::imwrite("/home/rosmatch/Bilder/dijkstra_visited.png", visited);
            return PlanningStatus::Success;
        }
        else if (curr_cell_x == goal.x && curr_cell_y == goal.y)
        {
            found_goal = true;
            continue;
        }
        for (auto delta : deltas)
        {
            // expansion
            int x2 = curr_cell_x + delta.x;
            int y2 = curr_cell_y + delta.y;
            double l1_cost = delta.x + delta.y;
            double l2_cost = sqrt(pow(delta.x, 2) + pow(delta.y, 2));

            // check new node to be in grid bounds
            if (x2 >= 0 && x2 < voronoi_map.rows && y2 >= 0 && y2 < voronoi_map.cols)
            {
                // check new node not to be in obstacle
                if (obstacle_map.at<uchar>(x2, y2) == 255)
                {
                    continue;
                }
                // check new node was not early visited
                if (visited.at<uchar>(x2, y2) == 255)
                {
                    continue;
                }

                float g2 = g + l2_cost; // movement_cost
                open_cells.push_back(std::make_tuple(g2, x2, y2));
                visited.at<uchar>(x2, y2) = 255;
            }
        }
    }
    return PlanningStatus::DirectConnectionToGoal;
}

bool findDijkstraPathOnVoronoi(const cv::Mat& voronoi_map, std::vector<cv::Point2i>& path, cv::Point2i start,
                               cv::Point2i goal, bool use_eight_neighbors)
{
    std::vector<cv::Point2i> deltas = plan_utils::create_deltas(use_eight_neighbors);
    cv::Mat visited(voronoi_map.size(), CV_8UC1, cv::Scalar(0));
    visited.at<uchar>(start) = 255;

    cv::Mat reached_with_delta_index(voronoi_map.size(), CV_8UC1, cv::Scalar(255));

    float g = 0;
    int movement_cost = 1;  // doesnt matter in which direction or angle, one cell movement costs 1

    // vector of open cells (for possible expansion) nodes
    std::vector<std::tuple<float, int, int>> open_cells;
    open_cells.push_back(std::make_tuple(g, start.x, start.y));

    // path found flag
    bool found_goal = false;

    while (!found_goal)
    {
        if (open_cells.size() == 0)
        {
            path.empty();
            return false;
        }
        // sort open by cost
        std::sort(open_cells.begin(), open_cells.end());
        std::reverse(open_cells.begin(), open_cells.end());
        // get node with lowest cost
        std::tuple<float, int, int> next = open_cells[open_cells.size() - 1];
        open_cells.pop_back();
        g = std::get<0>(next);
        int x = std::get<1>(next);
        int y = std::get<2>(next);

        // check, whether the solution is found (we are at the goal)
        if (x == goal.x && y == goal.y)
        {
            found_goal = true;
            continue;
        }
        for (int i = 0; i < deltas.size(); i++)
        {
            // expansion
            int x2 = x + deltas[i].x;
            int y2 = y + deltas[i].y;

            // check new node to be in grid bounds
            if (x2 >= 0 && x2 < voronoi_map.rows && y2 >= 0 && y2 < voronoi_map.cols)
            {
                // check new node not to be in obstacle
                if (voronoi_map.at<uchar>(x2, y2) == OBSTACLE_VALUE)
                {
                    continue;
                }
                // check new node was not early visited
                if (visited.at<uchar>(x2, y2) == 255)
                {
                    continue;
                }

                // check new node is on Voronoi diagram
                if (!(voronoi_map.at<uchar>(x2, y2) == VORONOI_VALUE))
                {
                    continue;
                }
                double l1_dist = abs(deltas[i].x) + abs(deltas[i].y);
                movement_cost = l1_dist;
                float g2 = g + movement_cost;
                open_cells.push_back(std::make_tuple(g2, x2, y2));
                visited.at<uchar>(x2, y2) = 255;
                reached_with_delta_index.at<uchar>(x2, y2) = i;
            }
        }
    }

    // Make reverse steps from goal to init to write path
    int x = goal.x;
    int y = goal.y;

    int i = 0;
    path.clear();

    while (x != start.x || y != start.y)
    {
        path.push_back(cv::Point2i(x, y));
        i++;

        int x2 = x - deltas[reached_with_delta_index.at<uchar>(x, y)].x;
        int y2 = y - deltas[reached_with_delta_index.at<uchar>(x, y)].y;

        x = x2;
        y = y2;
    }

    std::reverse(path.begin(), path.end());

    return true;
}
}  // namespace dijkstra_planning

