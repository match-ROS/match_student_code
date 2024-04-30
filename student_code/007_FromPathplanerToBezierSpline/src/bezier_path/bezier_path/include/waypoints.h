#include <utils.h>
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


struct Point;
bool linearPathClean(const std::vector<Point>& path, const nav_msgs::OccupancyGrid& map);
bool isObstacleInLine(const nav_msgs::OccupancyGrid& map, const Point& start, const Point& end);
bool ObstacleOnVectorPoint(const std::vector<Point>& vectorPoint, const nav_msgs::OccupancyGrid& map);
bool isPointObstacle(const nav_msgs::OccupancyGrid& map, const Point& point);

std::pair<std::vector<Point>, bool> LineOfSight(const nav_msgs::OccupancyGrid& map, const std::vector<Point>& receivedPath, const float& targetLength,
                               const Eigen::Vector2f& startTangentLocal, const Eigen::Vector2f& goalTangentLocal, const size_t& angle);
Point getDirection(const Point& a, const Point& b);
Point getNextPointInSight(const nav_msgs::OccupancyGrid& map, const std::vector<Point>& receivedPath,
                          const Point& comparePoint, const size_t& j);
void extendLineIncrementally(Point& basePoint, Point& changedPoint, const Point& direction, const float& stepSize);
Point extendLineByDistance(const Point& point, const Point& direction, const double& distance);
double pointDistance(const Point& p1, const Point& p2);
double minPointDistance(const std::vector<Point>& points);
double calculateAngle(const Point& a, const Point& b, const Point& c);
bool isBigAngle(const Point& a, const Point& b, const Point& c, double thresholdAngle);
Point movePoint(const Point& p, const Point& v);
bool isLeftTurn(const Point& a, const Point& b, const Point& c);
Point orthogonalDirectionRight(const Point& dir);
Point orthogonalDirectionLeft(const Point& dir);