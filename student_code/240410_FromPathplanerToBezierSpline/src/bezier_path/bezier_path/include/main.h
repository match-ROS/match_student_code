#include <waypoints.h>
#include <bezierpath.h>
#include <matlab.h>
#include <rviz.h>
#include <utils.h>
#include "std_msgs/String.h"

struct Point;

// callbacks
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void planCallback(const nav_msgs::Path::ConstPtr& msg);

// calculations
void calculateBezier(const std::vector<Point>& LoS, const float& targetSplineLength, const float& tangentFactor,
                     std::vector<Eigen::Vector2f>& bezier_spline,
                     std::vector<Eigen::Vector2f>& first_derivative_bezier_spline,
                     std::vector<Eigen::Vector2f>& second_derivative_bezier_spline,
                     std::vector<float>& curvature_bezier_spline, float& currMaxCurvature, const bool& analyseSpline);
void findBestPath();




//rviz
// void markerpublishPoint(const Point& point, const std::string& ns, const double& size, const double& r, const double& g, const double& b);
// void markerpublishEigenPoint(const Eigen::Vector2f& point, const std::string& ns, const double& size, const double& r, const double& g, const double& b);
// void markerpublishEigen(const std::vector<Eigen::Vector2f>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);
// void publishArrowEigen(const Eigen::Vector2f& position, const Eigen::Vector2f& direction, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);
// void publishPointsForControlPoints(const std::vector<Eigen::Vector2f>& points, const std::string& ns, double scale, double r, double g, double b);
// void markerpublish(std::vector<Point>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);
