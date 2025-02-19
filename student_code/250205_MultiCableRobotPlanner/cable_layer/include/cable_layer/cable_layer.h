// Copyright 2018 David V. Lu!!
#ifndef CABLE_LAYER_H_
#define CABLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/Range.h>
#include <cable_layer/CableLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include <cable_layer/CablePoints.h>
#include <actionlib/server/simple_action_server.h>
#include <cable_layer/ExchangeCablePointsAction.h>

namespace cable_layer_namespace
{

class CableLayer : public costmap_2d::CostmapLayer
{
public:
  CableLayer();  // Konstruktor
  virtual ~CableLayer();  // Destruktor

  virtual void onInitialize();
  
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  void sendCablePointsToOtherRobots();

private:
  void updateCablePoints(const ros::TimerEvent& event);
  void setCablePoint(double wx, double wy);
  void reconfigureCB(cable_layer::CableLayerConfig &config, uint32_t level);
  void resetRange();
  void updateCostmap();

  void publishCablePoints();
  void cablePointsCallback(const cable_layer::CablePoints::ConstPtr& msg);
  void executeCableExchangeCallback(const cable_layer::ExchangeCablePointsGoalConstPtr &goal);
  
  ros::Timer cable_points_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener;

  ros::Publisher cable_points_pub_;
  ros::Subscriber cable_points_sub_;
  
  actionlib::SimpleActionServer<cable_layer::ExchangeCablePointsAction> *action_server_;

  std::string robot_namespace_;

  std::list<std::pair<double, double>> cable_points_list_;
       
  double to_prob(unsigned char c)
  {
    return static_cast<double>(c) / costmap_2d::LETHAL_OBSTACLE;
  }
  unsigned char to_cost(double p)
  {
    return static_cast<unsigned char>(p * costmap_2d::LETHAL_OBSTACLE);
  }

  boost::mutex range_message_mutex_;
  std::list<sensor_msgs::Range> range_msgs_buffer_;
  std::map<std::pair<unsigned int, unsigned int>, double> marked_point_history_;

  double max_angle_, phi_v_;
  double inflate_cone_;
  std::string global_frame_;

  double clear_threshold_, mark_threshold_;
  bool clear_on_max_reading_;

  double no_readings_timeout_;
  ros::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<ros::Subscriber> range_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  bool use_decay_;
  double pixel_decay_;
  double transform_tolerance_;

  dynamic_reconfigure::Server<cable_layer::CableLayerConfig> *dsrv_;
};
}  // namespace cable_layer_namespace
#endif  // CABLE_LAYER_H_

