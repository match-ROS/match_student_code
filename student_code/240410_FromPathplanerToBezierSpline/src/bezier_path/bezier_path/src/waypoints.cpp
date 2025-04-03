#include <waypoints.h>

struct Point {
    double x, y;
    bool operator==(const Point& other) const { // overload == operator
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const { // overload != operator
        return !(*this == other);
    }
    Point operator*(float scalar) const {
        return {x * scalar, y * scalar};
    }
    void normalize() { // set vector length to 1
        double length = sqrt(pow(x, 2) + pow(y, 2));
        if (length != 0) {// avoid dividing by 0
            x /= length;
            y /= length;
        }
    }
    void negate() { // reverse the direction of the vector
        x = -x;
        y = -y;
    }
};

bool linearPathClean(const std::vector<Point>& path, const nav_msgs::OccupancyGrid& map) {
    if(path.empty()) {return true;}
    for(size_t i = 0; i < path.size()-1; i++) {
        if(isObstacleInLine(map, path[i],path[i+1])) {
            return false;
        }
    }
    return true;
}

Point getNextPointInSight(const nav_msgs::OccupancyGrid& map, const std::vector<Point>& receivedPath,
 const Point& comparePoint, const size_t& j) //comp = thisPointInSight
{
    Point localNextPointInSight;
    size_t m = receivedPath.size() - 1;
    while (m > j) { // j is where thisPointInSight is
        if (!isObstacleInLine(map, comparePoint, receivedPath[m])){
            localNextPointInSight = receivedPath[m];
            break;
        }
        else{m--;}
    }
    return localNextPointInSight;
}

bool ObstacleOnVectorPoint(const std::vector<Point>& vectorPoint, const nav_msgs::OccupancyGrid& map) {    
    if(vectorPoint.empty()) {return true;}
    for (const auto& point : vectorPoint) {
        if (isPointObstacle(map, point)) {
            return true;
        }
    }
    return false;
}

Point getDirection(const Point& a, const Point& b) {
    Point direction;
    // Berechne die Richtung von Punkt a zu Punkt b
    direction.x = b.x - a.x;
    direction.y = b.y - a.y;
    direction.normalize();
    return direction;
}

void extendLineIncrementally(Point& basePoint, Point& changedPoint, const Point& direction, const float& stepSize) {
    changedPoint.x = basePoint.x + direction.x * stepSize;
    changedPoint.y = basePoint.y + direction.y * stepSize;
}


Point extendLineByDistance(const Point& point, const Point& direction, const double& distance) {
    Point newPoint;
    newPoint.x = point.x + direction.x * distance;
    newPoint.y = point.y + direction.y * distance;
    return newPoint;
}

double pointDistance(const Point& p1, const Point& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

double minPointDistance(const std::vector<Point>& points) {
    double min_distance = std::numeric_limits<double>::max(); // Initialisiere mit maximalem Wert
    for (size_t i = 1; i < points.size(); ++i) { // Beginne bei 1, um das aktuelle und das vorherige Element zu vergleichen
        double distance = pointDistance(points[i - 1], points[i]);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

Point movePoint(const Point& p, const Point& v) { // move Point p by vector v
    return {p.x + v.x, p.y + v.y};
}

double distanceFromZero(const Point& a) {
    return sqrt(pow(a.x,2)+pow(a.y,2));
}

double calculateAngle(const Point& a, const Point& b, const Point& c) { // calculates angles of up to 180° between 3 points
    if (a == b || b == c) { // right and left turns so never above 180
        return 0.0;
    }
    Point ab = {b.x - a.x, b.y - a.y}; // create vectors
    Point cb = {b.x - c.x, b.y - c.y}; // reverse direction because angle is measured at middle point b

    double dot = ab.x * cb.x + ab.y * cb.y; // dotproduct
    double det = ab.x * cb.y - ab.y * cb.x; // determinant
    double angle = atan2(det, dot) * (180.0 / M_PI); // calculate angle in degrees

    if (angle < 0) { // always positive
        angle += 360.0;
    }
    if (angle > 180) { // right and left turns so never above 180
        angle = 360 - angle;
    }
    return angle;
}

bool isLeftTurn(const Point& a, const Point& b, const Point& c) {
    float vectorAB_x = b.x - a.x;
    float vectorAB_y = b.y - a.y;
    float vectorBC_x = c.x - b.x;
    float vectorBC_y = c.y - b.y;

    float crossProduct = vectorAB_x * vectorBC_y - vectorAB_y * vectorBC_x;
    if (crossProduct > 0) {
        return true; // left turn
    } else {
        return false; // right turn
    }
}

Point orthogonalDirectionRight(const Point& dir) {
    Point ortho_dir;
    ortho_dir.x = dir.y;
    ortho_dir.y = -dir.x;
    return ortho_dir;
}

Point orthogonalDirectionLeft(const Point& dir) {
    Point ortho_dir;
    ortho_dir.x = -dir.y;
    ortho_dir.y = dir.x;
    return ortho_dir;
}

bool isBigAngle(const Point& a, const Point& b, const Point& c, double thresholdAngle) { // returns if an angle between 3 points is bigger than the definied thresholdAngle
    double angle = calculateAngle(a, b, c);
    return angle > thresholdAngle;
}

bool isPointObstacle(const nav_msgs::OccupancyGrid& map, const Point& point) { // returns if Point is on an obstacle
    // calculate middle point of map
    double ox = map.info.width / 2.0 * map.info.resolution; // ox is half of map width in meters (world coordinates)
    double oy = map.info.height / 2.0 * map.info.resolution; // oy is half of map height in meters (world coordinates)

    // Transform world coordinates in map coordinates
    int x0 = static_cast<int>((point.x + ox) / map.info.resolution); 
    int y0 = static_cast<int>((point.y + oy) / map.info.resolution);
        
    int index = y0 * map.info.width + x0; //calculate index in OccupancyGrid
    // check if obstacle in current point
    if (map.data[index] > 0) { // everything above this is an obstacle
        return true; // result = obstacle found
    }
    return false; // result = no obstacle found
}

bool isObstacleInLine(const nav_msgs::OccupancyGrid& map, const Point& start, const Point& end) {
    Point direction = getDirection(start, end);
    double distance = pointDistance(start, end);
    double progress = 0.0;
    float step_size = 0.02;
    std::vector<Point> line;
    line.reserve(static_cast<size_t>(distance / step_size) + 1);
    Point point = start;
    line.push_back(start);

    while(progress < distance - step_size) {
        extendLineIncrementally(point, point, direction, step_size);
        line.push_back(point);
        progress += step_size;
    }
    if(line.back() != end) {line.push_back(end);}

    if(ObstacleOnVectorPoint(line, map)) {
        return true;
    }
    else {
        return false;
    }
}

std::pair<std::vector<Point>, bool> LineOfSight(const nav_msgs::OccupancyGrid& map, const std::vector<Point>& receivedPath, const float& targetLength,
                               const Eigen::Vector2f& startTangentLocal, const Eigen::Vector2f& goalTangentLocal, const size_t& angle) 
{
    double step_size = 0.01;
    bool lengthReached = true;
    std::vector<Point> LoSPath{}; // important that it is empty at start
    if (receivedPath.empty()) {
        ROS_INFO("No Path received to calculate LoSPath");
        return std::make_pair(LoSPath, lengthReached);
    }  
    size_t i = 0; // Starting point
    bool afterStartAdded = false;
    bool preEndBool = false;
    Point direction;
    Point nextPointInSight;
    Point extendedPoint;
    Point firstPoint;
    Point thisPointInSight;
    Point pointToSet;
    size_t z;

    Point startTangentPoint = eigenToPoint(startTangentLocal); // convert Eigen to Point
    startTangentPoint.normalize();

    Point goalTangentPoint = eigenToPoint(goalTangentLocal);
    goalTangentPoint.negate();
    goalTangentPoint.normalize();  

    Point afterStartPoint = extendLineByDistance(receivedPath.front(), startTangentPoint, targetLength);
    Point preEndPoint = extendLineByDistance(receivedPath.back(), goalTangentPoint, targetLength);
    

    if(isObstacleInLine(map, receivedPath.front(), afterStartPoint)) {
        ROS_INFO("Please don't start by driving into the wall");
        return std::make_pair(LoSPath, lengthReached);
    }
    if(isObstacleInLine(map, preEndPoint, receivedPath.back())) {
        ROS_INFO("Need more space to reach goal orientation");
        return std::make_pair(LoSPath, lengthReached);
    }

    LoSPath.push_back(receivedPath.front());
    firstPoint = receivedPath.front();


    while (i < receivedPath.size() - 1) {
        size_t j = receivedPath.size() - 1; // Start j at the end of path

        while (j > i) {              
            if (!isObstacleInLine(map, firstPoint, receivedPath[j])) { // if no obstacles between i and j
                thisPointInSight = receivedPath[j];
                if(thisPointInSight == receivedPath.back() 
                &&(preEndBool 
                ||(pointDistance(thisPointInSight, receivedPath.back()) > targetLength && isObstacleInLine(map, thisPointInSight, preEndPoint)))) {
                    LoSPath.push_back(thisPointInSight);
                    i = j;
                    break;
                }
                if(!afterStartAdded) { // can't be added directly because angle has to be checked
                    thisPointInSight = afterStartPoint;
                    afterStartAdded = true;
                }
                if(!isObstacleInLine(map, firstPoint, preEndPoint)
                // && calculateAngle(firstPoint, preEndPoint, receivedPath.back()) > angle // works better without
                && pointDistance(firstPoint, preEndPoint) > targetLength) { // preEndPoint in sight, has priority
                    LoSPath.push_back(preEndPoint);
                    LoSPath.push_back(receivedPath.back());
                    return std::make_pair(LoSPath, lengthReached);
                }           
                // else if(thisPointInSight != receivedPath.back()
                // && pointDistance(thisPointInSight, receivedPath.back()) < targetLength // handle the ending
                // && !isObstacleInLine(map, thisPointInSight, receivedPath.back()))
                // {                     
                //     preEndBool = true;
                //     Point dir = getDirection(thisPointInSight, firstPoint);
                //     Point prePoint = extendLineByDistance(thisPointInSight, dir, targetLength + 0.001 - pointDistance(thisPointInSight, receivedPath.back()));

                //     if(isObstacleInLine(map, prePoint, receivedPath.back()) || isObstacleInLine(map, firstPoint, prePoint)
                //     || pointDistance(firstPoint, prePoint) < targetLength) 
                //     {
                //         dir = getDirection(receivedPath.back(), thisPointInSight);
                //         prePoint = extendLineByDistance(thisPointInSight, dir, targetLength + 0.001 - pointDistance(thisPointInSight, receivedPath.back()));

                //         if(isObstacleInLine(map, prePoint, receivedPath.back()) || isObstacleInLine(map, firstPoint, prePoint)
                //         || pointDistance(firstPoint, prePoint) < targetLength)
                //         {                            
                //             // ROS_INFO("second to last point");
                //             lengthReached = false;
                //             LoSPath.clear();
                //             return std::make_pair(LoSPath, lengthReached);
                //         }
                //     }
                //     thisPointInSight = prePoint;
                // }
                pointToSet = thisPointInSight; // avoiding name confusion
                nextPointInSight = getNextPointInSight(map, receivedPath, thisPointInSight, j);

                // extension
                if(pointDistance(thisPointInSight, nextPointInSight) < targetLength) { // next Point not far enough
                    
                    direction = getDirection(firstPoint, thisPointInSight);
                    extendedPoint = thisPointInSight;

                    while(!isObstacleInLine(map, thisPointInSight, extendedPoint)
                    && !isObstacleInLine(map, extendedPoint, nextPointInSight)
                    && pointDistance(extendedPoint, nextPointInSight) < targetLength)
                    {                        
                        extendLineIncrementally(extendedPoint, extendedPoint, direction, step_size);
                        nextPointInSight = getNextPointInSight(map, receivedPath, extendedPoint, j);
                    }
                    if(isObstacleInLine(map, thisPointInSight, extendedPoint) || isObstacleInLine(map, extendedPoint, nextPointInSight)) {
                        
                        direction.negate(); // back to the non obstacle point
                        extendLineIncrementally(extendedPoint, extendedPoint, direction, step_size);
                        nextPointInSight = getNextPointInSight(map, receivedPath, extendedPoint, j);
                    }
                    if(pointDistance(extendedPoint, nextPointInSight) < targetLength) {
                        // ROS_INFO("targetLength too big");
                        lengthReached = false;
                        // LoSPath.clear();
                        // return std::make_pair(LoSPath, lengthReached);
                    }
                    pointToSet = extendedPoint; 
                }

                // reduce curvature by replacing one point with two
                if(calculateAngle(firstPoint, pointToSet, nextPointInSight) < angle) { 
                    double lengthToPrev = pointDistance(pointToSet, firstPoint);
                    Point tangentOne = pointToSet;
                    Point tangentTwo = pointToSet;
                    float extender = 0.0;
                    size_t reason;
                    Point dirPrev = getDirection(pointToSet, firstPoint); // normalized direction vector from pointToSet to firstPoint
                    Point dirNext = getDirection(pointToSet, nextPointInSight); 
                    
                    while(lengthToPrev > targetLength
                       && !isObstacleInLine(map, tangentOne, tangentTwo)
                       && pointDistance(tangentOne, tangentTwo) < targetLength)
                    {
                        extender += 0.01; 
                        tangentOne = movePoint(pointToSet, dirPrev * extender);
                        tangentTwo = movePoint(pointToSet, dirNext * extender);
                    }
                    // determine why the loop stopped
                    if(lengthToPrev < targetLength || lengthToPrev == targetLength) {reason = 1;}                   
                    if(isObstacleInLine(map, tangentOne, tangentTwo)) {reason = 2;}
                    if(pointDistance(tangentOne, tangentTwo) > targetLength || pointDistance(tangentOne, tangentTwo) == targetLength) {reason = 3;}
                    // two of these are also possible so they are ordered and no else if
                    // go back to the last value that worked
                    extender -= 0.01; 
                    tangentOne = movePoint(pointToSet, dirPrev * extender);
                    tangentTwo = movePoint(pointToSet, dirNext * extender);


                    switch(reason) {
                        case 1: {// lengthToPrev < targetLength
                            // std::cout<<"case 1"<<std::endl;                            
                            Point tangentDirection = getDirection(tangentOne, tangentTwo);// direction between tangentOne and tangentTwo
                            Point incrementedPoint = tangentTwo;
                            Point checkpoint = tangentTwo; // in case of obstacle reset extension
                            while(!isObstacleInLine(map, tangentOne, incrementedPoint)
                               && pointDistance(tangentOne, incrementedPoint) < targetLength) 
                            {
                                tangentTwo = incrementedPoint; // doesn't get set if conditions don't meet
                                extendLineIncrementally(tangentTwo, incrementedPoint, tangentDirection, step_size); // extend only in this direction because of reason 1
                            }
                            if(isObstacleInLine(map, tangentOne, incrementedPoint)) { // two cant be extended sufficiently so one has to be
                                // extended orthogonal to dirprev so distance to previous Point doesn't get smaller
                                // determine direction
                                // std::cout<<"case 1, One has to be extended orthogonally"<<std::endl;
                                tangentTwo = checkpoint;                                                
                                bool leftTurn = isLeftTurn(firstPoint, pointToSet, nextPointInSight); // extend away from curve direction
                                Point orthoDir;
                                if(leftTurn) {orthoDir = orthogonalDirectionLeft(dirPrev);}
                                else{orthoDir = orthogonalDirectionRight(dirPrev);}
                                
                                Point orthoPoint = tangentOne;
                                while(!isObstacleInLine(map, tangentTwo, orthoPoint)
                                   && pointDistance(tangentTwo, orthoPoint) < targetLength) 
                                {
                                    tangentOne = orthoPoint; // doesn't get set if conditions don't meet
                                    extendLineIncrementally(tangentOne, orthoPoint, orthoDir, step_size);
                                }
                                if(isObstacleInLine(map, tangentTwo, orthoPoint)) { // warning
                                    lengthReached = false;
                                    // std::cout<<"minDist couldn't be reached, distance is "<<pointDistance(tangentOne, tangentTwo)<<std::endl;
                                    LoSPath.clear();
                                    return std::make_pair(LoSPath, lengthReached);
                                }                              
                            }
                            break;
                        }
                        case 2: { // obstacle between tangentPoints, very similar approach to case 1
                            // std::cout<<"case 2"<<std::endl;
                            Point tangentDirectionTwo = getDirection(tangentOne, tangentTwo);
                            Point incrementedPointTwo = tangentTwo;
                            Point checkpoint = tangentTwo; // in case of obstacle reset extension
                            while(!isObstacleInLine(map, tangentOne, incrementedPointTwo) //extend Two
                               && pointDistance(tangentOne, incrementedPointTwo) < targetLength) 
                            {
                                tangentTwo = incrementedPointTwo; // doesn't get set if conditions don't meet
                                extendLineIncrementally(tangentTwo, incrementedPointTwo, tangentDirectionTwo, step_size); // extend only in this direction because of reason 1
                            }
                            if(isObstacleInLine(map, tangentOne, incrementedPointTwo)) { // two cant be extended further so one has to be
                                tangentTwo = checkpoint;
                                Point tangentDirectionOne = getDirection(tangentTwo, tangentOne);
                                Point incrementedPointOne = tangentOne;

                                while(!isObstacleInLine(map, tangentTwo, incrementedPointOne)
                                   && pointDistance(tangentTwo, incrementedPointOne) < targetLength
                                   && pointDistance(incrementedPointOne, firstPoint) > targetLength) 
                                {
                                    tangentOne = incrementedPointOne; // doesn't get set if conditions don't meet
                                    extendLineIncrementally(tangentOne, incrementedPointOne, tangentDirectionOne, step_size); // extend only in this direction because of reason 1
                                }
                                if(pointDistance(incrementedPointOne, firstPoint) < targetLength || pointDistance(incrementedPointOne, firstPoint) == targetLength) {
                                    // extended orthogonal to dirprev so distance to previous Point doesn't get smaller
                                    // determine direction
                                    // std::cout<<"case 2, One has to be extended orthogonally"<<std::endl;                        
                                    bool leftTurn = isLeftTurn(firstPoint, pointToSet, nextPointInSight); // extend away from curve direction
                                    Point orthoDir;

                                    if(leftTurn) {orthoDir = orthogonalDirectionLeft(dirPrev);}
                                    else{orthoDir = orthogonalDirectionRight(dirPrev);}
                                    
                                    Point orthoPoint = tangentOne;

                                    while(!isObstacleInLine(map, tangentTwo, orthoPoint)
                                       && pointDistance(tangentTwo, orthoPoint) < targetLength) 
                                    {
                                        tangentOne = orthoPoint; // doesn't get set if conditions don't meet
                                        extendLineIncrementally(tangentOne, orthoPoint, orthoDir, step_size);
                                    }
                                    if(isObstacleInLine(map, tangentTwo, orthoPoint)) { // warning
                                        lengthReached = false;                                       
                                        // std::cout<<"minDist couldn't be reached, distance is "<<pointDistance(tangentOne, tangentTwo)<<std::endl;
                                        LoSPath.clear();
                                        return std::make_pair(LoSPath, lengthReached);
                                    }
                                }                             
                            }
                            break;
                        }
                        case 3: { // pointDistance big enough
                            // std::cout<<"case 3"<<std::endl;                            
                            break; // no action needed
                        }
                    }
                    LoSPath.push_back(tangentOne);
                    LoSPath.push_back(tangentTwo);
                } // end of small angle measures
                else {
                    LoSPath.push_back(pointToSet);
                }
                firstPoint = LoSPath.back();
                i = j;
                if(pointToSet == afterStartPoint) {
                    i = 0;
                }
                break; // Break inner loop             
            } 
            else {                
                j--; // next Point
                if(j == i) {
                    // ROS_INFO("can't find next in sight");
                    LoSPath.clear();
                    return std::make_pair(LoSPath, lengthReached);
                }
            }
        }
    }
    if (LoSPath.back() != receivedPath.back()) {
        LoSPath.push_back(receivedPath.back()); // add ending Point if not already
    }
    // double minPointDist = minPointDistance(LoSPath);
    // std::cout<<"minPointDist: "<<minPointDist<<std::endl;
    return std::make_pair(LoSPath, lengthReached);
}

// old version, not accurate enough
// bool isObstacleInLine(const nav_msgs::OccupancyGrid& map, const Point& start, const Point& end) { // returns if an obstacle is in a straight line between 2 points
//     Point obstacle_point;
//     // calculate middle point of map
//     double ox = map.info.width / 2.0 * map.info.resolution; // ox is half of map width in meters (world coordinates)
//     double oy = map.info.height / 2.0 * map.info.resolution; // oy is half of map height in meters (world coordinates)

//     // Transform world coordinates in map coordinates
//     int x0 = static_cast<int>((start.x + ox) / map.info.resolution); 
//     int y0 = static_cast<int>((start.y + oy) / map.info.resolution); 
//     int x1 = static_cast<int>((end.x + ox) / map.info.resolution); 
//     int y1 = static_cast<int>((end.y + oy) / map.info.resolution); 

//     // Bresenham algorithm
//     int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
//     int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
//     int err = dx + dy, e2;

//     while (true) {        
//         int index = y0 * map.info.width + x0; //calculate index in OccupancyGrid
//         // check if obstacle in current point
//         if (map.data[index] > 0) { // everything above this is an obstacle
//             obstacle_point.x = x0 * map.info.resolution - ox; // Convert back to world coordinates
//             obstacle_point.y = y0 * map.info.resolution - oy;
//             std::cout<<"obstacle found at: "<<obstacle_point.x<<", "<<obstacle_point.y<<std::endl;
//             return true; // result = obstacle found in this line
//         }
//         if (x0 == x1 && y0 == y1) {
//             break; // goal reached
//         }
//         e2 = 2 * err;
//         if (e2 >= dy) {
//             err += dy; x0 += sx;
//         }
//         if (e2 <= dx) { 
//             err += dx; y0 += sy; 
//         }
//     }
//     return false; // result = no obstacle found in this line
// }