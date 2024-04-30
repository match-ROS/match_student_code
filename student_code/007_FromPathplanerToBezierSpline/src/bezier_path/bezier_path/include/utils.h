#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h" //initialPose
#include "geometry_msgs/PoseStamped.h" //goal
#include "nav_msgs/OccupancyGrid.h" //map
#include <stdlib.h> //abs()
#include <visualization_msgs/Marker.h> // RViz Linie zwischen zwei Punkten
#include <iostream> //test
#include <algorithm> //for std::max
#include <Eigen/Dense>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


struct Point;
Eigen::Vector2f pointToEigen(const Point& p);
std::vector<Eigen::Vector2f> convertPointsToEigenVector(const std::vector<Point>& points);
Point eigenToPoint(const Eigen::Vector2f& vec);
std::vector<Point> convertEigenVectorToPoints(const std::vector<Eigen::Vector2f>& eigenPoints);
void reduceInflation(const double& radius);
void reduceMapUpdates(const double& update_frequency, const double& publish_frequency);