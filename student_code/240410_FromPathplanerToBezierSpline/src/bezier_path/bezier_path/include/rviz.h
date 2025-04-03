#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

struct Point;
void markerpublishPoint(ros::Publisher& marker_pub, const Point& point, const std::string& ns, const double& size, const double& r, const double& g, const double& b);
void markerpublishEigenPoint(ros::Publisher& marker_pub, const Eigen::Vector2f& point, const std::string& ns, const double& size, const double& r, const double& g, const double& b);
void markerpublishEigen(ros::Publisher& marker_pub, const std::vector<Eigen::Vector2f>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);
void publishArrowEigen(ros::Publisher& marker_pub, const Eigen::Vector2f& position, const Eigen::Vector2f& direction, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);
void publishPointsForControlPoints(ros::Publisher& marker_pub, const std::vector<Eigen::Vector2f>& points, const std::string& ns, double scale, double r, double g, double b);
void markerpublish(ros::Publisher& marker_pub, std::vector<Point>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b);