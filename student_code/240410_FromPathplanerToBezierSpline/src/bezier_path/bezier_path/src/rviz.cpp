#include <rviz.h>

struct Point {
    double x, y;
};

void markerpublishPoint(ros::Publisher& marker_pub, const Point& point, const std::string& ns, const double& size,
                        const double& r, const double& g, const double& b)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0;

    marker_pub.publish(marker);
}

void markerpublishEigenPoint(ros::Publisher& marker_pub, const Eigen::Vector2f& point, const std::string& ns, const double& size, const double& r, const double& g, const double& b) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0;

    marker_pub.publish(marker);
}

void markerpublish(ros::Publisher& marker_pub, std::vector<Point>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time();
    line_strip.ns = ns;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = thickness;
    line_strip.color.r = r;
    line_strip.color.g = g;
    line_strip.color.b = b;
    line_strip.color.a = 1;
    for (const auto& point : vector) { // connect points
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        line_strip.points.push_back(p);
    }
    marker_pub.publish(line_strip);
}

void markerpublishEigen(ros::Publisher& marker_pub, const std::vector<Eigen::Vector2f>& vector, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time();
    line_strip.ns = ns;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = thickness;
    line_strip.color.r = r;
    line_strip.color.g = g;
    line_strip.color.b = b;
    line_strip.color.a = 1.0;
    for (const auto& point : vector) {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0;
        line_strip.points.push_back(p);
    }
    marker_pub.publish(line_strip);
}

void publishArrowEigen(ros::Publisher& marker_pub, const Eigen::Vector2f& position, const Eigen::Vector2f& direction, const std::string& ns, const double& thickness, const double& r, const double& g, const double& b) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time();
    arrow.ns = ns;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;

    arrow.scale.x = thickness;
    arrow.scale.y = thickness * 2;
    arrow.scale.z = thickness * 2;

    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::Point start;
    start.x = position[0];
    start.y = position[1];
    start.z = 0;

    geometry_msgs::Point end;
    end.x = position[0] + direction[0] * 0.5;
    end.y = position[1] + direction[1] * 0.5;
    end.z = 0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    marker_pub.publish(arrow);
}

void publishPointsForControlPoints(ros::Publisher& marker_pub, const std::vector<Eigen::Vector2f>& points, const std::string& ns, double scale, double r, double g, double b) {
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = ns;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.scale.x = scale;
    points_marker.scale.y = scale;
    points_marker.color.r = r;
    points_marker.color.g = g;
    points_marker.color.b = b;
    points_marker.color.a = 1.0;

    for (const auto& point : points) {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0;
        points_marker.points.push_back(p);
    }
    marker_pub.publish(points_marker);
}