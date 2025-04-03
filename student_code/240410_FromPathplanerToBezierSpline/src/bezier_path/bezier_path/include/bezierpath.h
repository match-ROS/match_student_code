#include <utils.h>
#include <memory>
#include <Eigen/Core>
#include <iostream>
#include <bezier_splines/quintic_bezier_spline_HW.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <tf/transform_datatypes.h> // for quaternions
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>
#include <utility>
#include <tf/tf.h>
#include <fstream>
#include "nav_msgs/OccupancyGrid.h" //map


struct Point;
bool isEigenObstacle(const nav_msgs::OccupancyGrid& map, const Eigen::Vector2f& point);
bool ObstacleOnBezierSpline(const std::vector<Eigen::Vector2f>& bezierSpline, const nav_msgs::OccupancyGrid& map);
std::vector<Eigen::Vector2f> similarLength(std::vector<Eigen::Vector2f> W, const float& targetLength);
double pointDistanceEigen(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);
double bezierSplineLength(const std::vector<Eigen::Vector2f>& bezierSpline, const nav_msgs::OccupancyGrid& map);

Eigen::Matrix2f calcRotationMatrix(const double& angle);
Eigen::Vector2f calcUl(const std::vector<Eigen::Vector2f>& W, double beta, const size_t& j, const bool& negate);
std::pair<double, bool> calculateAngle(const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c);
Eigen::Vector2f quaternionTo2DVector(const geometry_msgs::Quaternion& q);