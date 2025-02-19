#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <cable_layer/ExchangeCablePointsAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "exchange_cable_points_client");
	
    ros::NodeHandle nh("~");
    std::string robot_namespace;
    nh.param<std::string>("robot_namespace", robot_namespace, "/mir1"); // Standard ist mir1
    
    // Configure the action client with the correct namespace
    actionlib::SimpleActionClient<cable_layer::ExchangeCablePointsAction> ac(nh, robot_namespace + "/move_base_flex/exchange_cable_points", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    cable_layer::ExchangeCablePointsGoal goal;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}

