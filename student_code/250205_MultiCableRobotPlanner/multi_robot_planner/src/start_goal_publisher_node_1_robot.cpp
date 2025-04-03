#include <ros/ros.h>
#include <multi_robot_planner/StartGoalPairList.h>
#include <multi_robot_planner/StartGoalPair.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool planner_ready = false;
bool mir1_start_received = false;

geometry_msgs::PoseStamped mir1_start;

void plannerReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        planner_ready = true;
        ROS_INFO("GlobalPlanner is ready to receive start-goal pairs.");
    }
}

// Callback for the first robot
void mir1StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir1_start_received) {
        mir1_start = *msg;
        mir1_start_received = true;
        ROS_INFO("Received start position for mir1.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "start_goal_sender");
    ros::NodeHandle nh;

    ros::Publisher pairs_pub = nh.advertise<multi_robot_planner::StartGoalPairList>("/start_goal_pairs", 1);
    ros::Subscriber ready_sub = nh.subscribe("/global_planner/ready", 1, plannerReadyCallback);

    // Subscribers for the start positions of the robots
    ros::Subscriber mir1_sub = nh.subscribe("/mir1/mir_pose_stamped_simple", 1, mir1StartCallback);

    ros::Rate loop_rate(1);

    // Wait until global planer is ready
    while (ros::ok() && (!planner_ready || !mir1_start_received)) {
        ROS_INFO("Waiting for GlobalPlanner to be ready and for start positions...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check if global planner is ready and start points are received
    if (planner_ready && mir1_start_received) {
        multi_robot_planner::StartGoalPairList pair_list;
	
	// Pair 1
        multi_robot_planner::StartGoalPair pair1;
        pair1.robot_id = 1;
        pair1.stage = 1;
        pair1.start = mir1_start;
        pair1.goal.pose.position.x = 1.0;
        pair1.goal.pose.position.y = 0.0;
        pair1.goal.pose.orientation.x = 0.0;
        pair1.goal.pose.orientation.y = 0.0;
        pair1.goal.pose.orientation.z = 0.0;
        pair1.goal.pose.orientation.w = 1.0;
        pair_list.pairs.push_back(pair1);

        // Pair 2
        multi_robot_planner::StartGoalPair pair2;
        pair2.robot_id = 1;
        pair2.stage = 2;
        pair2.start = pair1.goal;
        pair2.goal.pose.position.x = 4.0;
        pair2.goal.pose.position.y = 0.0;
        pair2.goal.pose.orientation.x = 0.0;
        pair2.goal.pose.orientation.y = 0.0;
        pair2.goal.pose.orientation.z = 0.0;
        pair2.goal.pose.orientation.w = 1.0;
        pair_list.pairs.push_back(pair2);

        pairs_pub.publish(pair_list);
        ROS_INFO("Start-goal pairs sent.");
    }

    ros::spin();
    return 0;
}

