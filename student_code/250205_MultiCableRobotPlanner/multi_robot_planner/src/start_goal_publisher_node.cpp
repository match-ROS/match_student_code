#include <ros/ros.h>
#include <multi_robot_planner/StartGoalPairList.h>
#include <multi_robot_planner/StartGoalPair.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool planner_ready = false;

void plannerReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        planner_ready = true;
        ROS_INFO("GlobalPlanner is ready to receive start-goal pairs.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "start_goal_sender");
    ros::NodeHandle nh;

    ros::Publisher pairs_pub = nh.advertise<multi_robot_planner::StartGoalPairList>("/start_goal_pairs", 1);
    ros::Subscriber ready_sub = nh.subscribe("/global_planner/ready", 1, plannerReadyCallback);

    ros::Rate loop_rate(1);

    while (ros::ok() && !planner_ready) {
        ROS_INFO("Waiting for GlobalPlanner to be ready...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (planner_ready) {
        multi_robot_planner::StartGoalPairList pair_list;

        // Define and add the pairs
        multi_robot_planner::StartGoalPair pair1;
        pair1.robot_id = 1;
        pair1.stage = 1;
        pair1.start.pose.position.x = -2.0;
        pair1.start.pose.position.y = 2.0;
        pair1.start.pose.orientation.w = 1.0;
        pair1.goal.pose.position.x = 7.0;
        pair1.goal.pose.position.y = 0.0;
        pair1.goal.pose.orientation.x = 0.0;
        pair1.goal.pose.orientation.y = 0.0;
        pair1.goal.pose.orientation.z = 0.0;
        pair1.goal.pose.orientation.w = 1.0;
        pair_list.pairs.push_back(pair1);

        multi_robot_planner::StartGoalPair pair2;
        pair2.robot_id = 2;
        pair2.stage = 1;
        pair2.start.pose.position.x = -2.0;
        pair2.start.pose.position.y = 0.0;
        pair2.start.pose.orientation.w = 1.0;
        pair2.goal.pose.position.x = 4.0;
        pair2.goal.pose.position.y = 0.0;
        pair2.goal.pose.orientation.w = 1.0;
        pair_list.pairs.push_back(pair2);

        multi_robot_planner::StartGoalPair pair3;
        pair3.robot_id = 3;
        pair3.stage = 1;
        pair3.start.pose.position.x = -2.0;
        pair3.start.pose.position.y = -2.0;
        pair3.start.pose.orientation.w = 1.0;
        pair3.goal.pose.position.x = 1.0;
        pair3.goal.pose.position.y = 0.0;
        pair3.goal.pose.orientation.w = 1.0;
        pair_list.pairs.push_back(pair3);

        pairs_pub.publish(pair_list);
        ROS_INFO("Start-goal pairs sent.");
    }

    ros::spin();
    return 0;
}

