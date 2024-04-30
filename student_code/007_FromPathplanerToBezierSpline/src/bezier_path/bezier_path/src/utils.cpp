#include <utils.h>


struct Point {
    double x, y;
    bool operator==(const Point& other) const { // overload == operator
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const { // overload != operator
        return !(*this == other);
    }
};

// converting point to Eigen
Eigen::Vector2f pointToEigen(const Point& p) {
    return Eigen::Vector2f(p.x, p.y);
}

std::vector<Eigen::Vector2f> convertPointsToEigenVector(const std::vector<Point>& points) {
    std::vector<Eigen::Vector2f> eigenPoints;
    eigenPoints.reserve(points.size()); // Optimierung: Speicher im Voraus reservieren
    for (const Point& p : points) {
        eigenPoints.push_back(pointToEigen(p)); // Konvertieren und hinzufügen
    }
    return eigenPoints;
}

Point eigenToPoint(const Eigen::Vector2f& vec) {
    Point pt;
    pt.x = vec.x();
    pt.y = vec.y();
    return pt;
}

std::vector<Point> convertEigenVectorToPoints(const std::vector<Eigen::Vector2f>& eigenPoints) {
    std::vector<Point> points;
    points.reserve(eigenPoints.size()); // Optimierung: Speicher im Voraus reservieren
    for (const Eigen::Vector2f& vec : eigenPoints) {
        points.push_back(eigenToPoint(vec)); // Konvertieren und hinzufügen
    }
    return points;
}

void reduceInflation(const double& radius) {
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter set_param;
  dynamic_reconfigure::Config conf;

  set_param.name = "inflation_radius";
  set_param.value = radius;
  conf.doubles.push_back(set_param);
  srv_req.config = conf;

  if (ros::service::call("/move_base_node/global_costmap/inflation/set_parameters", srv_req, srv_resp)) {
    // ROS_INFO("call to set inflation_radius parameters succeeded");
  } else {
    // ROS_INFO("call to set inflation_radius parameters failed");
  }
}

void reduceMapUpdates(const double& update_frequency, const double& publish_frequency) { // reduce freq of map updates
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter update_param;
    dynamic_reconfigure::DoubleParameter publish_param;
    dynamic_reconfigure::Config conf;

    // Set update_frequency parameter
    update_param.name = "update_frequency";
    update_param.value = update_frequency;
    conf.doubles.push_back(update_param);

    // Set publish_frequency parameter
    publish_param.name = "publish_frequency";
    publish_param.value = publish_frequency;
    conf.doubles.push_back(publish_param);

    // Add parameters to the ReconfigureRequest
    srv_req.config = conf;

    // Call the service to set parameters
    if (ros::service::call("/move_base_node/global_costmap/set_parameters", srv_req, srv_resp)) {
        // ROS_INFO("Call to set update_frequency and publish_frequency parameters succeeded");
    } else {
        // ROS_INFO("Call to set update_frequency and publish_frequency parameters failed");
    }
}