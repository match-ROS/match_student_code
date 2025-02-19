// Copyright 2018 David V. Lu!!
#include "cable_layer/cable_layer.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include <cable_layer/CablePoints.h>
#include <geometry_msgs/Point.h>

PLUGINLIB_EXPORT_CLASS(cable_layer_namespace::CableLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace cable_layer_namespace
{

CableLayer::CableLayer()
  : tf_buffer_(), tf_listener(tf_buffer_)
{}

CableLayer::~CableLayer()
{}

void CableLayer::onInitialize()
{
  ROS_INFO("Method onInitialize called.");

  ros::NodeHandle nh("~/" + name_);
  robot_namespace_ = nh.getNamespace();
  
  // Initialize Publisher and Subscriber
  cable_points_pub_ = nh.advertise<cable_layer::CablePoints>("/exchange_cable_points", 10);
  cable_points_sub_ = nh.subscribe("/exchange_cable_points", 10, &CableLayer::cablePointsCallback, this);
  if (cable_points_pub_.getNumSubscribers() == 0) {
      ROS_WARN("No subscribers to 'cable_points' topic.");
  } else {
      ROS_INFO("Publisher 'cable_points' successfully initialized.");
  }
  
  action_server_ = new actionlib::SimpleActionServer<cable_layer::ExchangeCablePointsAction>(ros::NodeHandle("~"), "exchange_cable_points", boost::bind(&CableLayer::executeCableExchangeCallback, this, _1), false);
    action_server_->start();

  double update_frequency;
  nh.param("update_frequency", update_frequency, 25.0);
  cable_points_timer_ = nh.createTimer(ros::Duration(1.0 / update_frequency), &CableLayer::updateCablePoints, this);

  current_ = true;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);

  matchSize();
  resetRange();

  nh.param("use_decay", use_decay_, false);
  nh.param("pixel_decay", pixel_decay_, 10.0);
  nh.param("transform_tolerance_", transform_tolerance_, 0.3);

  dsrv_ = new dynamic_reconfigure::Server<cable_layer::CableLayerConfig>(nh);
  dynamic_reconfigure::Server<cable_layer::CableLayerConfig>::CallbackType cb =
    boost::bind(&CableLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();
}

double calculateYaw(const geometry_msgs::Quaternion& quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}

bool isPointNear(const std::pair<double, double>& pt1, const std::pair<double, double>& pt2, double epsilon)
{
    double dx = pt1.first - pt2.first;
    double dy = pt1.second - pt2.second;
    return (dx * dx + dy * dy) <= (epsilon * epsilon);
}

void CableLayer::updateCablePoints(const ros::TimerEvent&)
{
    geometry_msgs::TransformStamped transform;

    // Extract the robot name space
    std::string robot_name = robot_namespace_;
    size_t pos = robot_name.find("/move_base_flex/");
    if (pos != std::string::npos) {
        robot_name = robot_name.substr(0, pos); // Schneide alles nach dem Roboter-Namen ab
    }
	
    if (!robot_name.empty() && robot_name[0] == '/') {
        robot_name = robot_name.substr(1);
    }
	
    std::string base_link_frame = robot_name + "/base_link";

    try {
        transform = tf_buffer_.lookupTransform(global_frame_, base_link_frame, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(10, "Transform error in updateCablePoints: %s", ex.what());
        return;
    }

    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    double robot_yaw = calculateYaw(transform.transform.rotation);

    double cable_x = robot_x - 0.9 * cos(robot_yaw);
    double cable_y = robot_y - 0.9 * sin(robot_yaw);

    std::pair<double, double> new_point = std::make_pair(cable_x, cable_y);

    double epsilon = 0.01;

    bool point_exists = false;
    for (const auto& pt : cable_points_list_)
    {
        if (isPointNear(pt, new_point, epsilon))
        {
            point_exists = true;
            break;
        }
    }

    // Add cable point if it does not exist yet
    if (!point_exists)
    {
        cable_points_list_.emplace_back(cable_x, cable_y);
//        ROS_INFO("Added new cable point: x=%f, y=%f", cable_x, cable_y);
    }

    // Set the cable point on the costmap
    setCablePoint(cable_x, cable_y);
}

void CableLayer::cablePointsCallback(const cable_layer::CablePoints::ConstPtr& msg)
{
    ROS_INFO("[%s] Received %lu cable points.", robot_namespace_.c_str(), msg->points.size());

    // Iterate over all points of the message
    for (const auto& pt : msg->points)
    {
//        ROS_INFO("[%s] Processing point: x=%f, y=%f", robot_namespace_.c_str(), pt.x, pt.y);

        // Set every point in the costmap
        setCablePoint(pt.x, pt.y);
    }
}

void CableLayer::sendCablePointsToOtherRobots()
{
    ROS_INFO("Method sendCablePointsToOtherRobots called");
    cable_layer::CablePoints msg;

    // Use the cable points list
    for (const auto& point : cable_points_list_)
    {
        geometry_msgs::Point pt;
        pt.x = point.first;
        pt.y = point.second;
        pt.z = 0.0;
        msg.points.push_back(pt);
	
//        ROS_INFO("Sending point: x=%f, y=%f", pt.x, pt.y);
    }

    cable_points_pub_.publish(msg);

    ROS_INFO("Cable points sent to other robots. Total points sent: %lu", msg.points.size());
}

void CableLayer::executeCableExchangeCallback(const cable_layer::ExchangeCablePointsGoalConstPtr &goal)
{
    ROS_INFO("Action callback triggered.");

    sendCablePointsToOtherRobots();

    cable_layer::ExchangeCablePointsResult result;
    result.success = true;
    result.message = "Cable points exchanged successfully.";
    action_server_->setSucceeded(result);
}

void CableLayer::setCablePoint(double wx, double wy) {
    unsigned int mx, my;

    if (worldToMap(wx, wy, mx, my)) {
        setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
//        ROS_INFO("Cable point set in costmap: mx=%u, my=%u", mx, my);
    } else {
        ROS_WARN("Cable point out of map bounds: wx=%f, wy=%f", wx, wy);
    }
}

void CableLayer::reconfigureCB(cable_layer::CableLayerConfig &config, uint32_t level)
{
//  ROS_INFO("Method reconfigureCB called.");
  phi_v_ = config.phi;
  inflate_cone_ = config.inflate_cone;
  no_readings_timeout_ = config.no_readings_timeout;
  clear_threshold_ = config.clear_threshold;
  mark_threshold_ = config.mark_threshold;
  clear_on_max_reading_ = config.clear_on_max_reading;

  if (enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void CableLayer::updateCostmap()
{
//  ROS_INFO("Method updateCostmap called.");
  range_message_mutex_.lock();
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();
}

void CableLayer::resetRange()
{
//  ROS_INFO("Method resetRange called.");
  min_x_ = min_y_ =  std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void CableLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
//    ROS_INFO("Method updateBounds called.");
    if (layered_costmap_->isRolling())
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    updateCostmap();

    *min_x = std::min(*min_x, min_x_) - 1.0;
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);

    resetRange();

    if (!enabled_)
    {
        current_ = true;
        return;
    }
}

void CableLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
//  ROS_INFO("Method updateCosts called.");
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if (prob == costmap_2d::NO_INFORMATION)
      {
        it++;
        continue;
      }
      else if (prob > mark)
        current = costmap_2d::LETHAL_OBSTACLE;
      else if (prob < clear)
        current = costmap_2d::FREE_SPACE;
      else
      {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current)
        master_array[it] = current;
      it++;
    }
  }

  buffered_readings_ = 0;
  current_ = true;
}
}  // namespace cable_layer_namespace
