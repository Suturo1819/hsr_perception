/**
 * A simple client to test whether the door is open
 * @author: Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_perception_msgs/AnalyzeShelfStatusAction.h>

using namespace suturo_perception_msgs;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_door");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<AnalyzeShelfStatusAction> ac("analyze_shelf_status", true);

    ROS_INFO("Waiting for ShelfServer to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Object info server started. Sending goal...");
    // send a goal to the action
    AnalyzeShelfStatusGoal goal;
    goal.visualize = 1;
    ac.sendGoal(goal);

    ROS_INFO("Goal has been sent. Waiting for results (this may take a while)...");
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

    if (finished_before_timeout)
    {
        auto res = ac.getResult();
        if(res->door_open) {
            ROS_INFO("Door is open");
        } else {
            ROS_INFO("Door is shut");
        }
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}