#include <ros/ros.h>
#include <multi_robot_planner/StartGoalPairList.h>
#include <multi_robot_planner/StartGoalPair.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool planner_ready = false;
bool mir1_start_received = false;
bool mir2_start_received = false;
bool mir3_start_received = false;

geometry_msgs::PoseStamped mir1_start;
geometry_msgs::PoseStamped mir2_start;
geometry_msgs::PoseStamped mir3_start;

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

// Callback for the second robot
void mir2StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir2_start_received) {
        mir2_start = *msg;
        mir2_start_received = true;
        ROS_INFO("Received start position for mir2.");
    }
}

// Callback for the third robot
void mir3StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir3_start_received) {
        mir3_start = *msg;
        mir3_start_received = true;
        ROS_INFO("Received start position for mir3.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "start_goal_sender");
    ros::NodeHandle nh;

    ros::Publisher pairs_pub = nh.advertise<multi_robot_planner::StartGoalPairList>("/start_goal_pairs", 1);
    ros::Subscriber ready_sub = nh.subscribe("/global_planner/ready", 1, plannerReadyCallback);

    // Subscribers for the start positions of the robots
    ros::Subscriber mir1_sub = nh.subscribe("/mir1/mir_pose_stamped_simple", 1, mir1StartCallback);
    ros::Subscriber mir2_sub = nh.subscribe("/mir2/mir_pose_stamped_simple", 1, mir2StartCallback);
    ros::Subscriber mir3_sub = nh.subscribe("/mir3/mir_pose_stamped_simple", 1, mir3StartCallback);

    ros::Rate loop_rate(1);

    // Wait until global planer is ready
    while (ros::ok() && (!planner_ready || !mir1_start_received || !mir2_start_received || !mir3_start_received)) {
        ROS_INFO("Waiting for GlobalPlanner to be ready and for start positions...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check if global planner is ready and start points are received
    if (planner_ready && mir1_start_received && mir2_start_received && mir3_start_received) {
        multi_robot_planner::StartGoalPairList pair_list;

        // Stage 1
            // Pair 1
            multi_robot_planner::StartGoalPair pair1;
            pair1.robot_id = 1;
            pair1.stage = 1;
            pair1.start = mir1_start;
            pair1.goal.pose.position.x = -7.0;
            pair1.goal.pose.position.y = 12.0;
            pair1.goal.pose.orientation.x = 0.0;
            pair1.goal.pose.orientation.y = 0.0;
            pair1.goal.pose.orientation.z = -0.7071;
            pair1.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair1);

            // Pair 2
            multi_robot_planner::StartGoalPair pair2;
            pair2.robot_id = 2;
            pair2.stage = 1;
            pair2.start = mir2_start;
            pair2.goal.pose.position.x = 0.0;
            pair2.goal.pose.position.y = 15.0;
            pair2.goal.pose.orientation.x = 0.0;
            pair2.goal.pose.orientation.y = 0.0;
            pair2.goal.pose.orientation.z = -0.7071;
            pair2.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair2);

            // Pair 3
            multi_robot_planner::StartGoalPair pair3;
            pair3.robot_id = 3;
            pair3.stage = 1;
            pair3.start = mir3_start;
            pair3.goal.pose.position.x = 7.0;
            pair3.goal.pose.position.y = 12.0;
            pair3.goal.pose.orientation.x = 0.0;
            pair3.goal.pose.orientation.y = 0.0;
            pair3.goal.pose.orientation.z = -0.7071;
            pair3.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair3);

        // Stage 2
            // Pair 4
            multi_robot_planner::StartGoalPair pair4;
            pair4.robot_id = 1;
            pair4.stage = 2;
            pair4.start = pair1.goal;
            pair4.goal.pose.position.x = 12.0;
            pair4.goal.pose.position.y = -7.0;
            pair4.goal.pose.orientation.x = 0.0;
            pair4.goal.pose.orientation.y = 0.0;
            pair4.goal.pose.orientation.z = -1.0;
            pair4.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair4);

            // Pair 5
            multi_robot_planner::StartGoalPair pair5;
            pair5.robot_id = 2;
            pair5.stage = 2;
            pair5.start = pair2.goal;
            pair5.goal.pose.position.x = 15.0;
            pair5.goal.pose.position.y = 0.0;
            pair5.goal.pose.orientation.x = 0.0;
            pair5.goal.pose.orientation.y = 0.0;
            pair5.goal.pose.orientation.z = -1.0;
            pair5.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair5);

            // Pair 6
            multi_robot_planner::StartGoalPair pair6;
            pair6.robot_id = 3;
            pair6.stage = 2;
            pair6.start = pair3.goal;
            pair6.goal.pose.position.x = 12.0;
            pair6.goal.pose.position.y = 7.0;
            pair6.goal.pose.orientation.x = 0.0;
            pair6.goal.pose.orientation.y = 0.0;
            pair6.goal.pose.orientation.z = -1.0;
            pair6.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair6);

        // Stage 3
            // Pair 7
            multi_robot_planner::StartGoalPair pair7;
            pair7.robot_id = 1;
            pair7.stage = 3;
            pair7.start = pair4.goal;
            pair7.goal.pose.position.x = 7.0;
            pair7.goal.pose.position.y = -12.0;
            pair7.goal.pose.orientation.x = 0.0;
            pair7.goal.pose.orientation.y = 0.0;
            pair7.goal.pose.orientation.z = 0.7071;
            pair7.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair7);

            // Pair 8
            multi_robot_planner::StartGoalPair pair8;
            pair8.robot_id = 2;
            pair8.stage = 3;
            pair8.start = pair5.goal;
            pair8.goal.pose.position.x = 0.0;
            pair8.goal.pose.position.y = -15.0;
            pair8.goal.pose.orientation.x = 0.0;
            pair8.goal.pose.orientation.y = 0.0;
            pair8.goal.pose.orientation.z = 0.7071;
            pair8.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair8);

            // Pair 9
            multi_robot_planner::StartGoalPair pair9;
            pair9.robot_id = 3;
            pair9.stage = 3;
            pair9.start = pair6.goal;
            pair9.goal.pose.position.x = -7.0;
            pair9.goal.pose.position.y = -12.0;
            pair9.goal.pose.orientation.x = 0.0;
            pair9.goal.pose.orientation.y = 0.0;
            pair9.goal.pose.orientation.z = 0.7071;
            pair9.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair9);

        pairs_pub.publish(pair_list);
        ROS_INFO("Start-goal pairs sent.");
    }

    ros::spin();
    return 0;
}

