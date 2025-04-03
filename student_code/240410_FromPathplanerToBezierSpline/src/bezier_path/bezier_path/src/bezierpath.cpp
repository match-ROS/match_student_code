#include <bezierpath.h>
#include <waypoints.h>

struct Point {
    double x, y;
};

bool isEigenObstacle(const nav_msgs::OccupancyGrid& map, const Eigen::Vector2f& point) { // returns if Eigen::Vector2f is on an obstacle
    // calculate middle point of map
    double ox = map.info.width / 2.0 * map.info.resolution; // ox is half of map width in meters (world coordinates)
    double oy = map.info.height / 2.0 * map.info.resolution; // oy is half of map height in meters (world coordinates)

    // Transform world coordinates in map coordinates
    int x0 = static_cast<int>((point.x() + ox) / map.info.resolution); 
    int y0 = static_cast<int>((point.y() + oy) / map.info.resolution); 
        
    int index = y0 * map.info.width + x0; //calculate index in OccupancyGrid
    // check if obstacle in current point
    if (map.data[index] > 0) { // everything above this is an obstacle
        return true; // result = obstacle found
    }
    return false; // result = no obstacle found
}

bool ObstacleOnBezierSpline(const std::vector<Eigen::Vector2f>& bezierSpline, const nav_msgs::OccupancyGrid& map) {
    if(bezierSpline.empty()) {return true;}
    size_t count=0;            
    for (const auto& point : bezierSpline) {
        if (isEigenObstacle(map, point)) {           
            count++;
        }
        else if(count > 0) {
            count--;
        }
        if(count > 0){
            return true;
        }
    }
    return false;
}

double pointDistanceEigen(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
    return std::sqrt((p2.x() - p1.x()) * (p2.x() - p1.x()) + (p2.y() - p1.y()) * (p2.y() - p1.y()));
}

double bezierSplineLength(const std::vector<Eigen::Vector2f>& bezierSpline, const nav_msgs::OccupancyGrid& map) {
    if(bezierSpline.empty()) {return 0.0;}            
    double distance = 0.0;
    for(size_t i = 0; i < bezierSpline.size()-1; i++) {
        distance += pointDistanceEigen(bezierSpline[i],bezierSpline[i+1]);
    }
    return distance;
}

std::vector<Eigen::Vector2f> similarLength(std::vector<Eigen::Vector2f> W, const float& targetLength) {
    if (W.size() < 2) {
        return W;
    }

    std::vector<Eigen::Vector2f> result;
    for (size_t i = 0; i < W.size() - 1; ++i) {
        result.push_back(W[i]);
        float dist = (W[i+1] - W[i]).norm();

        int numPointsToInsert = std::floor(dist / (1.0 * targetLength)) - 1; // as close as possible to x times minDistance without subpassing it

        Eigen::Vector2f step = (W[i+1] - W[i]) / (numPointsToInsert + 1);        
        for (int j = 0; j < numPointsToInsert; ++j) {
            result.push_back(W[i] + step * (j + 1));
        }
    }
    result.push_back(W.back());
    for (size_t i = 0; i < result.size() - 1; ++i) { // print distances
        float dist = (result[i+1] - result[i]).norm();
    }
    return result;
}

// calculations for bezier spline
Eigen::Vector2f calcUl(const std::vector<Eigen::Vector2f>& W, double beta, const size_t& j, const bool& negate) { // robot orientation at a certain point
    if(!negate) { // consider right and left corners
        beta = beta * (-1);       
    } 
    Eigen::Matrix2f R = calcRotationMatrix(beta);    
    Eigen::Vector2f u_l = R * (W[j+1] - W[j]) / ((W[j+1] - W[j]).norm()); // / ((W[j+1] - W[j]).norm()) sets length to 1
    return u_l;
}

Eigen::Matrix2f calcRotationMatrix(const double& angle) { // angle in degrees
    Eigen::Matrix2f R;
    double rad = angle * M_PI / 180.0;
    R << std::cos(rad), -std::sin(rad),
        std::sin(rad),  std::cos(rad);
    return R;
}

std::pair<double, bool> calculateAngle(const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c) {
    Eigen::Vector2f ab = b - a;
    Eigen::Vector2f cb = b - c;
    double dot = ab.dot(cb);
    double det = ab.x() * cb.y() - ab.y() * cb.x();
    double angle = std::atan2(det, dot) * (180.0 / M_PI);
    bool negate = true; // rechtskurve

    if (angle < 0) {
        angle += 360.0;
    }
    if (angle > 180) {
        angle = 360 - angle;
        negate = false; // linkskurve
    }
    return std::make_pair(angle, negate); // consider right and left corners
}

Eigen::Vector2f quaternionTo2DVector(const geometry_msgs::Quaternion& q) { // start and end orientations
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return Eigen::Vector2f(std::cos(yaw), std::sin(yaw));
}