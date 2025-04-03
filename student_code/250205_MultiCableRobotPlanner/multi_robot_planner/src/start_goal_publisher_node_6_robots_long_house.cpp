#include <ros/ros.h>
#include <multi_robot_planner/StartGoalPairList.h>
#include <multi_robot_planner/StartGoalPair.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool planner_ready = false;
bool mir1_start_received = false;
bool mir2_start_received = false;
bool mir3_start_received = false;
bool mir4_start_received = false;
bool mir5_start_received = false;
bool mir6_start_received = false;

geometry_msgs::PoseStamped mir1_start;
geometry_msgs::PoseStamped mir2_start;
geometry_msgs::PoseStamped mir3_start;
geometry_msgs::PoseStamped mir4_start;
geometry_msgs::PoseStamped mir5_start;
geometry_msgs::PoseStamped mir6_start;

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

// Callback for the fourth robot
void mir4StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir4_start_received) {
        mir4_start = *msg;
        mir4_start_received = true;
        ROS_INFO("Received start position for mir4.");
    }
}

// Callback for the fifth robot
void mir5StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir5_start_received) {
        mir5_start = *msg;
        mir5_start_received = true;
        ROS_INFO("Received start position for mir5.");
    }
}

// Callback for the sixth robot
void mir6StartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!mir6_start_received) {
        mir6_start = *msg;
        mir6_start_received = true;
        ROS_INFO("Received start position for mir6.");
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
    ros::Subscriber mir4_sub = nh.subscribe("/mir4/mir_pose_stamped_simple", 1, mir4StartCallback);
    ros::Subscriber mir5_sub = nh.subscribe("/mir5/mir_pose_stamped_simple", 1, mir5StartCallback);
    ros::Subscriber mir6_sub = nh.subscribe("/mir6/mir_pose_stamped_simple", 1, mir6StartCallback);

    ros::Rate loop_rate(1);

    // Wait until global planer is ready
    while (ros::ok() && (!planner_ready || !mir1_start_received || !mir2_start_received || !mir3_start_received || !mir4_start_received || !mir5_start_received || !mir6_start_received)) {
        ROS_INFO("Waiting for GlobalPlanner to be ready and for start positions...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check if global planner is ready and start points are received
    if (planner_ready && mir1_start_received && mir2_start_received && mir3_start_received && mir4_start_received && mir5_start_received && mir6_start_received) {
        multi_robot_planner::StartGoalPairList pair_list;

        // Stage 1
            // Pair 1
            multi_robot_planner::StartGoalPair pair1;
            pair1.robot_id = 1;
            pair1.stage = 1;
            pair1.start = mir1_start;
            pair1.goal.pose.position.x = -12.0;
            pair1.goal.pose.position.y = 0.0;
            pair1.goal.pose.orientation.x = 0.0;
            pair1.goal.pose.orientation.y = 0.0;
            pair1.goal.pose.orientation.z = 0.0;
            pair1.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair1);

            // Pair 2
            multi_robot_planner::StartGoalPair pair2;
            pair2.robot_id = 2;
            pair2.stage = 1;
            pair2.start = mir2_start;
            pair2.goal.pose.position.x = -14.0;
            pair2.goal.pose.position.y = 2.0;
            pair2.goal.pose.orientation.x = 0.0;
            pair2.goal.pose.orientation.y = 0.0;
            pair2.goal.pose.orientation.z = 0.0;
            pair2.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair2);

            // Pair 3
            multi_robot_planner::StartGoalPair pair3;
            pair3.robot_id = 3;
            pair3.stage = 1;
            pair3.start = mir3_start;
            pair3.goal.pose.position.x = -15.0;
            pair3.goal.pose.position.y = 4.5;
            pair3.goal.pose.orientation.x = 0.0;
            pair3.goal.pose.orientation.y = 0.0;
            pair3.goal.pose.orientation.z = 0.0;
            pair3.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair3);

            // Pair 4
            multi_robot_planner::StartGoalPair pair4;
            pair4.robot_id = 4;
            pair4.stage = 1;
            pair4.start = mir4_start;
            pair4.goal.pose.position.x = -15.0;
            pair4.goal.pose.position.y = 7.5;
            pair4.goal.pose.orientation.x = 0.0;
            pair4.goal.pose.orientation.y = 0.0;
            pair4.goal.pose.orientation.z = 0.0;
            pair4.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair4);

            // Pair 5
            multi_robot_planner::StartGoalPair pair5;
            pair5.robot_id = 5;
            pair5.stage = 1;
            pair5.start = mir5_start;
            pair5.goal.pose.position.x = -14.0;
            pair5.goal.pose.position.y = 10.0;
            pair5.goal.pose.orientation.x = 0.0;
            pair5.goal.pose.orientation.y = 0.0;
            pair5.goal.pose.orientation.z = 0.0;
            pair5.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair5);

            // Pair 6
            multi_robot_planner::StartGoalPair pair6;
            pair6.robot_id = 6;
            pair6.stage = 1;
            pair6.start = mir6_start;
            pair6.goal.pose.position.x = -12.0;
            pair6.goal.pose.position.y = 12.0;
            pair6.goal.pose.orientation.x = 0.0;
            pair6.goal.pose.orientation.y = 0.0;
            pair6.goal.pose.orientation.z = 0.0;
            pair6.goal.pose.orientation.w = 1.0;
            pair_list.pairs.push_back(pair6);

        // Stage 2
            // Pair 7
            multi_robot_planner::StartGoalPair pair7;
            pair7.robot_id = 1;
            pair7.stage = 2;
            pair7.start = pair1.goal;
            pair7.goal.pose.position.x = 7.0;
            pair7.goal.pose.position.y = 17.0;
            pair7.goal.pose.orientation.x = 0.0;
            pair7.goal.pose.orientation.y = 0.0;
            pair7.goal.pose.orientation.z = -0.7071;
            pair7.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair7);

            // Pair 8
            multi_robot_planner::StartGoalPair pair8;
            pair8.robot_id = 2;
            pair8.stage = 2;
            pair8.start = pair2.goal;
            pair8.goal.pose.position.x = 4.5;
            pair8.goal.pose.position.y = 19.0;
            pair8.goal.pose.orientation.x = 0.0;
            pair8.goal.pose.orientation.y = 0.0;
            pair8.goal.pose.orientation.z = -0.7071;
            pair8.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair8);

            // Pair 9
            multi_robot_planner::StartGoalPair pair9;
            pair9.robot_id = 3;
            pair9.stage = 2;
            pair9.start = pair3.goal;
            pair9.goal.pose.position.x = 1.5;
            pair9.goal.pose.position.y = 20.0;
            pair9.goal.pose.orientation.x = 0.0;
            pair9.goal.pose.orientation.y = 0.0;
            pair9.goal.pose.orientation.z = -0.7071;
            pair9.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair9);

            // Pair 10
            multi_robot_planner::StartGoalPair pair10;
            pair10.robot_id = 4;
            pair10.stage = 2;
            pair10.start = pair4.goal;
            pair10.goal.pose.position.x = -1.5;
            pair10.goal.pose.position.y = 20.0;
            pair10.goal.pose.orientation.x = 0.0;
            pair10.goal.pose.orientation.y = 0.0;
            pair10.goal.pose.orientation.z = -0.7071;
            pair10.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair10);

            // Pair 11
            multi_robot_planner::StartGoalPair pair11;
            pair11.robot_id = 5;
            pair11.stage = 2;
            pair11.start = pair5.goal;
            pair11.goal.pose.position.x = -4.5;
            pair11.goal.pose.position.y = 19.0;
            pair11.goal.pose.orientation.x = 0.0;
            pair11.goal.pose.orientation.y = 0.0;
            pair11.goal.pose.orientation.z = -0.7071;
            pair11.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair11);

            // Pair 12
            multi_robot_planner::StartGoalPair pair12;
            pair12.robot_id = 6;
            pair12.stage = 2;
            pair12.start = pair6.goal;
            pair12.goal.pose.position.x = -7.0;
            pair12.goal.pose.position.y = 17.0;
            pair12.goal.pose.orientation.x = 0.0;
            pair12.goal.pose.orientation.y = 0.0;
            pair12.goal.pose.orientation.z = -0.7071;
            pair12.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair12);

        // Stage 3
            // Pair 13
            multi_robot_planner::StartGoalPair pair13;
            pair13.robot_id = 1;
            pair13.stage = 3;
            pair13.start = pair7.goal;
            pair13.goal.pose.position.x = 12.0;
            pair13.goal.pose.position.y = 12.0;
            pair13.goal.pose.orientation.x = 0.0;
            pair13.goal.pose.orientation.y = 0.0;
            pair13.goal.pose.orientation.z = -1.0;
            pair13.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair13);

            // Pair 14
            multi_robot_planner::StartGoalPair pair14;
            pair14.robot_id = 2;
            pair14.stage = 3;
            pair14.start = pair8.goal;
            pair14.goal.pose.position.x = 14.0;
            pair14.goal.pose.position.y = 10.0;
            pair14.goal.pose.orientation.x = 0.0;
            pair14.goal.pose.orientation.y = 0.0;
            pair14.goal.pose.orientation.z = -1.0;
            pair14.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair14);

            // Pair 15
            multi_robot_planner::StartGoalPair pair15;
            pair15.robot_id = 3;
            pair15.stage = 3;
            pair15.start = pair9.goal;
            pair15.goal.pose.position.x = 15.0;
            pair15.goal.pose.position.y = 7.5;
            pair15.goal.pose.orientation.x = 0.0;
            pair15.goal.pose.orientation.y = 0.0;
            pair15.goal.pose.orientation.z = -1.0;
            pair15.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair15);

            // Pair 16
            multi_robot_planner::StartGoalPair pair16;
            pair16.robot_id = 4;
            pair16.stage = 3;
            pair16.start = pair10.goal;
            pair16.goal.pose.position.x = 15.0;
            pair16.goal.pose.position.y = 4.5;
            pair16.goal.pose.orientation.x = 0.0;
            pair16.goal.pose.orientation.y = 0.0;
            pair16.goal.pose.orientation.z = -1.0;
            pair16.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair16);

            // Pair 17
            multi_robot_planner::StartGoalPair pair17;
            pair17.robot_id = 5;
            pair17.stage = 3;
            pair17.start = pair11.goal;
            pair17.goal.pose.position.x = 14.0;
            pair17.goal.pose.position.y = 2.0;
            pair17.goal.pose.orientation.x = 0.0;
            pair17.goal.pose.orientation.y = 0.0;
            pair17.goal.pose.orientation.z = -1.0;
            pair17.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair17);

            // Pair 18
            multi_robot_planner::StartGoalPair pair18;
            pair18.robot_id = 6;
            pair18.stage = 3;
            pair18.start = pair12.goal;
            pair18.goal.pose.position.x = 13.0;
            pair18.goal.pose.position.y = 0.0;
            pair18.goal.pose.orientation.x = 0.0;
            pair18.goal.pose.orientation.y = 0.0;
            pair18.goal.pose.orientation.z = -1.0;
            pair18.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair18);

        // Stage 4
            // Pair 19
            multi_robot_planner::StartGoalPair pair19;
            pair19.robot_id = 1;
            pair19.stage = 4;
            pair19.start = pair13.goal;
            pair19.goal.pose.position.x = 12.0;
            pair19.goal.pose.position.y = -12.0;
            pair19.goal.pose.orientation.x = 0.0;
            pair19.goal.pose.orientation.y = 0.0;
            pair19.goal.pose.orientation.z = -1.0;
            pair19.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair19);

            // Pair 20
            multi_robot_planner::StartGoalPair pair20;
            pair20.robot_id = 2;
            pair20.stage = 4;
            pair20.start = pair14.goal;
            pair20.goal.pose.position.x = 14.0;
            pair20.goal.pose.position.y = -10.0;
            pair20.goal.pose.orientation.x = 0.0;
            pair20.goal.pose.orientation.y = 0.0;
            pair20.goal.pose.orientation.z = -1.0;
            pair20.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair20);

            // Pair 21
            multi_robot_planner::StartGoalPair pair21;
            pair21.robot_id = 3;
            pair21.stage = 4;
            pair21.start = pair15.goal;
            pair21.goal.pose.position.x = 15.0;
            pair21.goal.pose.position.y = -7.5;
            pair21.goal.pose.orientation.x = 0.0;
            pair21.goal.pose.orientation.y = 0.0;
            pair21.goal.pose.orientation.z = -1.0;
            pair21.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair21);

            // Pair 22
            multi_robot_planner::StartGoalPair pair22;
            pair22.robot_id = 4;
            pair22.stage = 4;
            pair22.start = pair16.goal;
            pair22.goal.pose.position.x = 15.0;
            pair22.goal.pose.position.y = -4.5;
            pair22.goal.pose.orientation.x = 0.0;
            pair22.goal.pose.orientation.y = 0.0;
            pair22.goal.pose.orientation.z = -1.0;
            pair22.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair22);

            // Pair 23
            multi_robot_planner::StartGoalPair pair23;
            pair23.robot_id = 5;
            pair23.stage = 4;
            pair23.start = pair17.goal;
            pair23.goal.pose.position.x = 14.0;
            pair23.goal.pose.position.y = -2.0;
            pair23.goal.pose.orientation.x = 0.0;
            pair23.goal.pose.orientation.y = 0.0;
            pair23.goal.pose.orientation.z = -1.0;
            pair23.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair23);

            // Pair 24
            multi_robot_planner::StartGoalPair pair24;
            pair24.robot_id = 6;
            pair24.stage = 4;
            pair24.start = pair18.goal;
            pair24.goal.pose.position.x = 13.0;
            pair24.goal.pose.position.y = 0.0;
            pair24.goal.pose.orientation.x = 0.0;
            pair24.goal.pose.orientation.y = 0.0;
            pair24.goal.pose.orientation.z = -1.0;
            pair24.goal.pose.orientation.w = 0.0;
            pair_list.pairs.push_back(pair24);

        // Stage 5
            // Pair 25
            multi_robot_planner::StartGoalPair pair25;
            pair25.robot_id = 1;
            pair25.stage = 5;
            pair25.start = pair19.goal;
            pair25.goal.pose.position.x = 7.0;
            pair25.goal.pose.position.y = -17.0;
            pair25.goal.pose.orientation.x = 0.0;
            pair25.goal.pose.orientation.y = 0.0;
            pair25.goal.pose.orientation.z = 0.7071;
            pair25.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair25);

            // Pair 26
            multi_robot_planner::StartGoalPair pair26;
            pair26.robot_id = 2;
            pair26.stage = 5;
            pair26.start = pair20.goal;
            pair26.goal.pose.position.x = 4.5;
            pair26.goal.pose.position.y = -19.0;
            pair26.goal.pose.orientation.x = 0.0;
            pair26.goal.pose.orientation.y = 0.0;
            pair26.goal.pose.orientation.z = 0.7071;
            pair26.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair26);

            // Pair 27
            multi_robot_planner::StartGoalPair pair27;
            pair27.robot_id = 3;
            pair27.stage = 5;
            pair27.start = pair21.goal;
            pair27.goal.pose.position.x = 1.5;
            pair27.goal.pose.position.y = -20.0;
            pair27.goal.pose.orientation.x = 0.0;
            pair27.goal.pose.orientation.y = 0.0;
            pair27.goal.pose.orientation.z = 0.7071;
            pair27.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair27);

            // Pair 28
            multi_robot_planner::StartGoalPair pair28;
            pair28.robot_id = 4;
            pair28.stage = 5;
            pair28.start = pair22.goal;
            pair28.goal.pose.position.x = -1.5;
            pair28.goal.pose.position.y = -20.0;
            pair28.goal.pose.orientation.x = 0.0;
            pair28.goal.pose.orientation.y = 0.0;
            pair28.goal.pose.orientation.z = 0.7071;
            pair28.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair28);

            // Pair 29
            multi_robot_planner::StartGoalPair pair29;
            pair29.robot_id = 5;
            pair29.stage = 5;
            pair29.start = pair23.goal;
            pair29.goal.pose.position.x = -4.5;
            pair29.goal.pose.position.y = -19.0;
            pair29.goal.pose.orientation.x = 0.0;
            pair29.goal.pose.orientation.y = 0.0;
            pair29.goal.pose.orientation.z = 0.7071;
            pair29.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair29);

            // Pair 30
            multi_robot_planner::StartGoalPair pair30;
            pair30.robot_id = 6;
            pair30.stage = 5;
            pair30.start = pair24.goal;
            pair30.goal.pose.position.x = -7.0;
            pair30.goal.pose.position.y = -17.0;
            pair30.goal.pose.orientation.x = 0.0;
            pair30.goal.pose.orientation.y = 0.0;
            pair30.goal.pose.orientation.z = 0.7071;
            pair30.goal.pose.orientation.w = 0.7071;
            pair_list.pairs.push_back(pair30);

        pairs_pub.publish(pair_list);
        ROS_INFO("Start-goal pairs sent.");
    }

    ros::spin();
    return 0;
}

