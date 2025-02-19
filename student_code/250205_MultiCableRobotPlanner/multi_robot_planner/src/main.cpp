#include <ros/ros.h>
#include "multi_robot_planner/global_planner_node.h"

int main(int argc, char** argv) {
    // Initialize the global planner node
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    GlobalPlanner planner(nh);

    planner.waitForCostmap();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        planner.processPairs();
        rate.sleep();
    }

    return 0;
}

