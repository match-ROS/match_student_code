#include <main.h>


geometry_msgs::PoseWithCovarianceStamped initial_pose;
geometry_msgs::PoseStamped goal;
nav_msgs::OccupancyGrid globalMap;
bool map_received = false;
bool map_ready = false;
bool path_received = false;
bool path_complete = false;
ros::Publisher marker_pub; //rviz visualization publisher
std::vector<Point> globalPath; // path planner path
Eigen::Vector2f goal_tangent;
Eigen::Vector2f start_tangent;



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
        // calculate lenght of vector
        double length = sqrt(pow(x, 2) + pow(y, 2));

        // avoid dividing by 0
        if (length != 0) {
            x /= length;
            y /= length;
        }
    }
    void negate() { // reverse the direction of the vector
        x = -x;
        y = -y;
    }
};

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    initial_pose = *msg;
    start_tangent = quaternionTo2DVector(initial_pose.pose.pose.orientation);
    start_tangent.normalize();
    ROS_INFO("Initialpose received");
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal = *msg;
    goal_tangent = quaternionTo2DVector(goal.pose.orientation);
    goal_tangent.normalize();
    ROS_INFO("Goal received");
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { // set globalMap
    if (map_ready) {             
        return;
    }
    if(!map_received) {
        globalMap = *msg;
        ROS_INFO("Map received with resolution %f, width %d, height %d", msg->info.resolution, msg->info.width, msg->info.height);
        map_received = true;
    }
    if(path_complete) {
        globalMap = *msg;
        ros::NodeHandle nh;
        double radius;
        if(!linearPathClean(globalPath, globalMap)) {
            nh.getParam("/move_base_node/global_costmap/inflation/inflation_radius", radius);
            radius -= 0.01;            
            reduceInflation(radius);
        }
        else {
            map_ready = true;
            nh.getParam("/move_base_node/global_costmap/inflation/inflation_radius", radius);
            std::cout<<"Inflationradius after: "<<radius<<std::endl;
            reduceMapUpdates(0.0, 0.0); // no more map updates needed 
            findBestPath();      
        }
    }
}

void planCallback(const nav_msgs::Path::ConstPtr& msg) { // saves Path of Pathplanner and calls controlpoints functions
    if (path_received) {// once a path is received, ignore new paths        
        return;
    }
    path_received = true;
    ROS_INFO("Path received");
    Point point;
    point.x = initial_pose.pose.pose.position.x;
    point.y = initial_pose.pose.pose.position.y;
    globalPath.push_back(point);
    for (const auto& pose : msg->poses) { // iterate through poses of the received msg 
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        globalPath.push_back(point);
    }
    point.x = goal.pose.position.x; // last point
    point.y = goal.pose.position.y;
    globalPath.push_back(point);
    ROS_INFO("Path complete");
    markerpublish(marker_pub, globalPath,"globalPath",0.025,1,1,0);
    path_complete = true;    
}

void findBestPath() {
    std::vector<Point> LoSPath; // waypoint vector
    
    // vectors and values of bezier-spline
    std::vector<Eigen::Vector2f> bezier_spline;
    std::vector<Eigen::Vector2f> first_derivative_bezier_spline;
    std::vector<Eigen::Vector2f> second_derivative_bezier_spline;
    std::vector<float> curvature_bezier_spline;
    float currMaxCurvature;

    // final vectors and values of bezier-spline
    float maxCurvature; 
    std::vector<Eigen::Vector2f> best_spline;

    // execution parameters
    bool measureCalcTime = true;
    bool analyseSpline = false;
    float tangentFactor = 2; // tF
    float tL_tF_relation = 1.0; // tangentFactor / targetSplineLength; 1.0 default
    float targetSplineLength = tangentFactor * tL_tF_relation; // tL
    size_t detail = 5; // 1: one try; 2: descending tangentFactors, stops at first solution; 3: also different targetLengths;
                       // 4: find lowest maxCurvature with static tL_tF_relation; 5: search for shortest path with maxCurvature constraint
    float tF_reduction = 0.1; // for cases 2, 3, 4 and 5
    float maxCurvatureLimit = 20.0; // for case 5


    
    bool solution = false;
    size_t angle = 90;
    std::pair<std::vector<Point>, bool> LoSResult;
    bool lengthReached;
    bool bezierClean = false;
    float bezierLength;
    start_tangent *= tangentFactor;
    goal_tangent *= tangentFactor;
    auto start_complete_time = std::chrono::steady_clock::now(); 
    
    switch (detail)
    {
        case 1: { // one try with tangentFactor as targetLength
            LoSResult = LineOfSight(globalMap, globalPath, tangentFactor, start_tangent, goal_tangent, angle);
            LoSPath = LoSResult.first;
            lengthReached = LoSResult.second;
            if(LoSPath.empty()) {break;}
            calculateBezier(LoSPath, targetSplineLength, tangentFactor, best_spline, first_derivative_bezier_spline,
                            second_derivative_bezier_spline, curvature_bezier_spline, currMaxCurvature, analyseSpline);
            if(!ObstacleOnBezierSpline(best_spline, globalMap)) {
                solution = true;
                maxCurvature = currMaxCurvature;
            }
            break;
        }
        case 2: { // try descending tangentFactors, stops at first obstacle free spline
            while((LoSPath.empty() || !bezierClean) && tangentFactor > 0.2) {
                targetSplineLength = tangentFactor * tL_tF_relation; // adjust tL to reduced tF
                LoSResult = LineOfSight(globalMap, globalPath, targetSplineLength, start_tangent, goal_tangent, angle);
                LoSPath = LoSResult.first;
                if(LoSPath.empty()) {
                    tangentFactor -= 0.1;
                    continue;
                }
                calculateBezier(LoSPath, targetSplineLength, tangentFactor, bezier_spline, first_derivative_bezier_spline,
                                second_derivative_bezier_spline, curvature_bezier_spline, currMaxCurvature, analyseSpline);
                bezierClean = !ObstacleOnBezierSpline(bezier_spline, globalMap);
                if(!bezierClean) {tangentFactor -= 0.1;}
                else {
                    best_spline = bezier_spline;
                    maxCurvature = currMaxCurvature;
                    solution = true;
                }
            }
            break;
        }
        case 3: { // try different tLs for each tF, tFs descend if no solution is found
            // extra vectors and values because calculations continue after solution is found        
            std::vector<Point> testPath; 
            float testSplineLength = tangentFactor * 0.7; // lower limit of realistic relation
            float maxCurvatureToSurpass = std::numeric_limits<float>::max(); // first comparison always true
            while(tangentFactor  > 0.2 && !solution) {   
                while(testSplineLength < (tangentFactor * 1.3)) { // figure out splineLength that leads to lowest curvature for one tangentFactor    

                    LoSResult = LineOfSight(globalMap, globalPath, testSplineLength, start_tangent, goal_tangent, angle);
                    testPath = LoSResult.first;
                    lengthReached = LoSResult.second;

                    if(testPath.empty()) {
                        if(solution) {break;}
                        if(testSplineLength + 0.01 < (tangentFactor * 1.3)) { // upper limit of realistic relation
                            testSplineLength += 0.01;
                        }
                        else{break;}
                        // std::cout << "testPath empty, testSplineLength increased to " << testSplineLength << std::endl;
                        continue;
                    }
                    calculateBezier(testPath, testSplineLength, tangentFactor, bezier_spline, first_derivative_bezier_spline,
                                    second_derivative_bezier_spline, curvature_bezier_spline, currMaxCurvature, analyseSpline);
                    if(currMaxCurvature < maxCurvatureToSurpass && !ObstacleOnBezierSpline(bezier_spline, globalMap)) {
                        solution = true; // while loop will continue anyway
                        maxCurvatureToSurpass = currMaxCurvature;
                        // std::cout << "new lowest Maximum curvature: " << maxCurvatureToSurpass << std::endl;
                        // std::cout << "according spline length: " << targetSplineLength << std::endl;

                        // save potential final values and vectors
                        LoSPath = testPath;
                        best_spline = bezier_spline;
                        maxCurvature = currMaxCurvature;
                        targetSplineLength = testSplineLength;
                    }
                    testSplineLength += 0.04;
                }
                tangentFactor -= tF_reduction;    
            }
            tangentFactor += tF_reduction; // correct tF after while condition fails
            break;
        }
        case 4: { // case 2 but don't stop at first solution to find lowest maxCurvature path
            // extra vectors and values because calculations continue after solution is found        
            float maxCurvatureToSurpass = std::numeric_limits<float>::max();
            std::vector<Point> testPath;
            float testTangentFactor = tangentFactor;
            float testTargetSplineLength = tangentFactor * tL_tF_relation;
            
            while(testTangentFactor  > 0.2) {
                testTargetSplineLength = testTangentFactor * tL_tF_relation;
                LoSResult = LineOfSight(globalMap, globalPath, testTargetSplineLength, start_tangent, goal_tangent, angle);
                testPath = LoSResult.first;
                lengthReached = LoSResult.second;
                if(testPath.empty()) {
                    testTangentFactor -= 0.1;
                    continue;
                }
                calculateBezier(testPath, testTargetSplineLength, testTangentFactor, bezier_spline, first_derivative_bezier_spline,
                                second_derivative_bezier_spline, curvature_bezier_spline, currMaxCurvature, analyseSpline);
                if(currMaxCurvature < maxCurvatureToSurpass && !ObstacleOnBezierSpline(bezier_spline, globalMap)) {
                    solution = true;
                    maxCurvatureToSurpass = currMaxCurvature;
                    // std::cout << "new lowest Maximum curvature: " << maxCurvatureToSurpass << std::endl;
                    // std::cout << "according tangentFactor: " << testTangentFactor << std::endl;
                    
                    // save potential final values and vectors                    
                    LoSPath = testPath;
                    best_spline = bezier_spline;
                    maxCurvature = currMaxCurvature;  
                    tangentFactor = testTangentFactor;
                    targetSplineLength  = testTargetSplineLength;         
                }
                testTangentFactor -= tF_reduction;
            }
            break;
        }
        case 5: { // finds shortest path for given curvature constraint, tF descends
            // extra vectors and values because calculations continue after solution is found        
            float currLength, lowestLength = std::numeric_limits<float>::max(); // first comparison always true
            std::vector<Point> testPath;
            float testTangentFactor = tangentFactor;
            float testTargetSplineLength = tangentFactor * tL_tF_relation;
            
            while(testTangentFactor  > 0.2) {
                testTargetSplineLength = testTangentFactor * tL_tF_relation;
                LoSResult = LineOfSight(globalMap, globalPath, testTargetSplineLength, start_tangent, goal_tangent, angle);
                testPath = LoSResult.first;
                lengthReached = LoSResult.second;
                if(testPath.empty()) {
                    testTangentFactor -= tF_reduction;
                    continue;
                }
                calculateBezier(testPath, testTargetSplineLength, testTangentFactor, bezier_spline, first_derivative_bezier_spline,
                                second_derivative_bezier_spline, curvature_bezier_spline, currMaxCurvature, analyseSpline);
                currLength = bezierSplineLength(bezier_spline, globalMap);
                if(currMaxCurvature < maxCurvatureLimit && !ObstacleOnBezierSpline(bezier_spline, globalMap)
                && currLength < lowestLength) {
                    solution = true;
                    lowestLength = currLength;
                    
                    // save potential final values and vectors                    
                    LoSPath = testPath;
                    best_spline = bezier_spline;
                    maxCurvature = currMaxCurvature;  
                    tangentFactor = testTangentFactor;
                    targetSplineLength  = testTargetSplineLength;         
                }
                testTangentFactor -= tF_reduction;
            }
            break;
        }
    }
    if (solution) { 
        // calculation time      
        auto end_complete_time = std::chrono::steady_clock::now();
        if(measureCalcTime) {
            auto duration_complete = std::chrono::duration_cast<std::chrono::milliseconds>(end_complete_time - start_complete_time);
            std::cout << "calculation time for complete curve with detail degree "<<detail<<" is "<< duration_complete.count() << " milliseconds" << std::endl;
        }

        // rviz visualisation
        markerpublish(marker_pub, LoSPath,"LoS",0.025,1,0.5,0);
        markerpublishEigen(marker_pub, best_spline, "best_spline", 0.025,0,1,1);

        ROS_INFO("spline data and parameters:");
        bezierLength = bezierSplineLength(best_spline, globalMap);
        std::cout<<"spline length: "<<bezierLength<<std::endl;
        std::cout<<"targetSplineLength: "<<targetSplineLength<<std::endl;
        std::cout<<"tangentFactor: "<<tangentFactor<<std::endl;
        std::cout<<"maximum curvature: "<<maxCurvature<<std::endl;
        if(detail == 3) {std::cout<<"tF / tL = "<<tangentFactor / targetSplineLength<<std::endl;}

        // comment in if needed, analyseSpline has to be true
        // matlabData(bezier_spline, first_derivative_bezier_spline, second_derivative_bezier_spline, curvature_bezier_spline, analyse);

        ROS_INFO("bezier-spline calculated");
    }
    else {
        ROS_INFO("failed to find a spline");
    }
}

void calculateBezier(const std::vector<Point>& LoS, const float& targetSplineLength, const float& tangentFactor,
                     std::vector<Eigen::Vector2f>& bezier_spline,
                     std::vector<Eigen::Vector2f>& first_derivative_bezier_spline,
                     std::vector<Eigen::Vector2f>& second_derivative_bezier_spline,
                     std::vector<float>& curvature_bezier_spline, float& currMaxCurvature, const bool& analyseSpline) 
{  
    // if called multiple times
    bezier_spline.clear();
    first_derivative_bezier_spline.clear();
    second_derivative_bezier_spline.clear();
    curvature_bezier_spline.clear();

    //preliminaries
    std::vector<Eigen::Vector2f> W = convertPointsToEigenVector(LoS); // waypoints in Eigen::Vector2f
    W = similarLength(W, targetSplineLength); // make length of segments close to tL
    size_t W_size = W.size();
    int resolution = 100; // amount of points per segment
    size_t j = 1; // Segment counter
    
    bezier_spline.reserve(static_cast<size_t>(W.size()-1) * 100 + 1);
    curvature_bezier_spline.reserve(static_cast<size_t>(W.size()-1) * 100 + 1);
    if(analyseSpline) {
        first_derivative_bezier_spline.reserve(static_cast<size_t>(W.size()-1) * 100 + 1);
        second_derivative_bezier_spline.reserve(static_cast<size_t>(W.size()-1) * 100 + 1);
    }
    
    // every spline
    Eigen::Vector2f start_pose_curr;
    Eigen::Vector2f start_pose_next;
    Eigen::Vector2f end_pose_curr;
    Eigen::Vector2f end_pose_next;
    Eigen::Vector2f start_tangent_curr;
    Eigen::Vector2f start_tangent_next;
    Eigen::Vector2f u_l_unit;
    Eigen::Vector2f end_tangent_curr;
    Eigen::Vector2f u_l_next_unit;
    Eigen::Vector2f end_tangent_next;
    double gamma_curr;
    double beta_curr;
    double gamma_next;
    double beta_next;
    bool negate;

    // controlpoint visualization
    std::vector<std::tuple<double, double, double>> colors = {
        {1.0, 0.0, 0.0}, // red
        {0.0, 1.0, 0.0}, // green
        {0.0, 0.0, 1.0}, // blue
        {1.0, 1.0, 0.0}, // yellow
        {0.0, 1.0, 1.0}, // cyan
        {1.0, 0.0, 1.0}  // violet
    };
    
    // initial values, orientation from python file
    start_pose_curr = {initial_pose.pose.pose.position.x,initial_pose.pose.pose.position.y};       
    end_pose_curr = {W[j]};
    start_pose_next = end_pose_curr;
    start_tangent_curr = start_tangent;
    std::tie(gamma_curr, negate) = calculateAngle(W[j-1], W[j], W[j+1]);		
    beta_curr = (180 - gamma_curr) / 2;
    u_l_unit = calcUl(W, beta_curr , j, negate);
    end_tangent_curr = u_l_unit * tangentFactor;
    start_tangent_next = end_tangent_curr;

    // next values
    std::tie(gamma_next, negate) = calculateAngle(W[j], W[j+1], W[j+2]);
    beta_next = (180 - gamma_next) / 2;
    u_l_next_unit = calcUl(W, beta_next, j+1, negate);
    end_tangent_next = u_l_next_unit * tangentFactor;
    end_pose_next = W[j+1];

    // final values, orientation of python file already calculated

    auto current_spline = std::make_shared<bezier_splines::QuinticBezierSplines>
        (W, j, start_pose_curr, start_tangent_curr, end_pose_curr, end_tangent_curr, goal_tangent);
    
    auto next_spline = std::make_shared<bezier_splines::QuinticBezierSplines>
        (W, j, start_pose_next, start_tangent_next, end_pose_next, end_tangent_next, goal_tangent);
    current_spline->setMemberNextSpline(next_spline);
    
    for(j; j < W_size; j++) { // loop to iterate through segments
        //following two lines = actual calculation
        current_spline->calcControlPoints();
        auto spline_segment = current_spline->calcBezierSpline(resolution);

        bezier_spline.insert(bezier_spline.end(), spline_segment.begin(), spline_segment.end());
        
        
        size_t last_element = 100;        
        if(j == W.size()-1) { //account for very last element of spline
            last_element = 101;
        } 
        std::vector<float> curvature_spline_segment;
        for(size_t i = 0; i < last_element; i++) {
            float j = static_cast<float>(i) / 100.0f;
            curvature_spline_segment.push_back(current_spline->calcCurvation(j));
        }
        curvature_bezier_spline.insert(curvature_bezier_spline.end(),curvature_spline_segment.begin(),curvature_spline_segment.end());

        if(analyseSpline) { // optional data   
            std::vector<Eigen::Vector2f> first_derivative_spline_segment;        
            for(size_t i = 0; i < last_element; i++) {
                float j = static_cast<float>(i) / 100.0f;
                first_derivative_spline_segment.push_back(current_spline->calcFirstDerivativeValue(j));
            }
            first_derivative_bezier_spline.insert(first_derivative_bezier_spline.end(),first_derivative_spline_segment.begin(),first_derivative_spline_segment.end());

            std::vector<Eigen::Vector2f> second_derivative_spline_segment;
            for(size_t i = 0; i < last_element; i++) {
                float j = static_cast<float>(i) / 100.0f;
                second_derivative_spline_segment.push_back(current_spline->calcSecondDerivativeValue(j));
            }
            second_derivative_bezier_spline.insert(second_derivative_bezier_spline.end(),second_derivative_spline_segment.begin(),second_derivative_spline_segment.end());
        }

        // rviz visu
        // std::string ns = "tangentArrow" + std::to_string(j);
        // std::string nsEnd = "EndtangentArrow" + std::to_string(j);
        // std::string nsCP = "CP" + std::to_string(j);

        // controlpoints change color for every segment
        // auto& color = colors[j % colors.size()];
        // double r = std::get<0>(color);
        // double g = std::get<1>(color);
        // double b = std::get<2>(color);
        // auto controlPoints = current_spline->getControlPoints();
        // publishPointsForControlPoints(controlPoints, nsCP, 0.25, r, g, b); // visualize cp
        // ros::Duration(0.05).sleep();
        // std::cout<<"CP0 curr: "<<controlPoints[0].x()<<", "<<controlPoints[0].y()<<std::endl;
        // std::cout<<"CP1 curr: "<<controlPoints[1].x()<<", "<<controlPoints[1].y()<<std::endl;
        // std::cout<<"CP2 curr: "<<controlPoints[2].x()<<", "<<controlPoints[2].y()<<std::endl;
        // std::cout<<"CP3 curr: "<<controlPoints[3].x()<<", "<<controlPoints[3].y()<<std::endl;
        // std::cout<<"CP4 curr: "<<controlPoints[4].x()<<", "<<controlPoints[4].y()<<std::endl;
        // std::cout<<"CP5 curr: "<<controlPoints[5].x()<<", "<<controlPoints[5].y()<<std::endl;
        // std::cout<<std::endl;
        
        // publishArrowEigen(W[j-1], current_spline->getStartTangent(), ns, 0.05, 1 ,0 , 0);
        // publishArrowEigen(W[j], current_spline->getEndTangent(), nsEnd, 0.06, 0, 1 , 1);

        //update spline Links
        if (j == W.size()-2) { // last update        
            current_spline->updatePreviousSpline(current_spline); // give previous_spline_ the attributes of current_spline_
            current_spline->updateCurrentSpline(next_spline);// give current_spline_ the attributes of next_spline_
            current_spline->setNextSpline(nullptr); // last segment doesn't have a next_spline_
        } 
        if (j == W.size()-3) { // second to last update, no end_tangent_next
            current_spline->updatePreviousSpline(current_spline); // give previous_spline_ the attributes of current_spline_
            current_spline->updateCurrentSpline(next_spline); // give current_spline_ the attributes of next_spline_
            
            // calc next_spline
            std::tie(gamma_curr, negate) = calculateAngle(W[j], W[j+1], W[j+2]);            
            beta_curr = (180 - gamma_curr) / 2;            
            u_l_unit = calcUl(W,beta_curr , j+1, negate);
            end_tangent_curr = u_l_unit * tangentFactor;

            // set changed Attributes for next_spline_
            next_spline->setStartPose(W[j+1]);
            next_spline->setEndPose(W[j+2]);
            next_spline->setStartTangent(end_tangent_curr); // current end tangent = next start tangent
            next_spline->setEndTangent(goal_tangent); // value from already calculated unter final values
        }  
        if (j < W.size()-3) { 
            current_spline->updatePreviousSpline(current_spline); // give previous_spline_ the attributes of current_spline_
            current_spline->updateCurrentSpline(next_spline); // give current_spline_ the attributes of next_spline_

            // calc next_spline
            std::tie(gamma_curr, negate) = calculateAngle(W[j], W[j+1], W[j+2]);
            beta_curr = (180 - gamma_curr) / 2;
            u_l_unit = calcUl(W,beta_curr , j+1, negate);
            end_tangent_curr = u_l_unit * tangentFactor;

            std::tie(gamma_next, negate)  = calculateAngle(W[j+1], W[j+2], W[j+3]);
            beta_next = (180 - gamma_next) / 2;
            u_l_next_unit = calcUl(W, beta_next, j+2, negate);
            end_tangent_next = u_l_next_unit * tangentFactor;

            // set changed Attributes for next_spline_
            next_spline->setStartPose(W[j+1]);
            next_spline->setEndPose(W[j+2]);
            next_spline->setStartTangent(end_tangent_curr); // current end tangent = next start tangent
            next_spline->setEndTangent(end_tangent_next);
        }
        // j_ iterates in for-loop	
    } 
    auto max_iter = std::max_element(curvature_bezier_spline.begin(), curvature_bezier_spline.end());
    size_t max_index = std::distance(curvature_bezier_spline.begin(), max_iter);// Get the index of the maximum element  
    currMaxCurvature = *max_iter;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_to_plan_node");
    ros::NodeHandle nh;
    ROS_INFO("THE MAIN");

    // ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber map_sub = nh.subscribe("/move_base_node/global_costmap/costmap", 1, mapCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, initialPoseCallback); // save starting point in plan
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback); // save goal in plan
    ros::Subscriber plan_sub = nh.subscribe("/move_base_node/NavfnROS/plan", 1, planCallback); // subscribe to path planner
    // ros::Subscriber plan_sub = nh.subscribe("/move_base_node/RAstarPlannerROS/global_plan", 1, planCallback);
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1); // global variable to advertise marker for rviz

    ros::spin(); 
    
    return 0;
}