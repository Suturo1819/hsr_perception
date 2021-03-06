/**
 * A naive client to test perception actions
 * Just for testing purposes
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_perception_msgs/PerceiveTableAction.h>

using namespace suturo_perception_msgs;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_perception");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<PerceiveTableAction> ac("hsr_perception_table", true);

    ROS_INFO("Waiting for Table action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Table Action server started, sending goal.");
    // send a goal to the action
    PerceiveTableGoal goal;
    goal.visualisation = 1;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

    if (finished_before_timeout)
    {
        auto res = ac.getResult();
        for(auto obj : res->detectionData) {
            ROS_INFO("==================================================");
            ROS_INFO("Detected object: %s", obj.name.c_str());
            ROS_INFO("x: %f", obj.pose.pose.position.x);
            ROS_INFO("y: %f", obj.pose.pose.position.y);
            ROS_INFO("z: %f", obj.pose.pose.position.z);
            ROS_INFO("Width: %f", obj.width);
            ROS_INFO("Height: %f", obj.height);
            ROS_INFO("Depth: %f", obj.depth);
            ROS_INFO("Shape: %d", obj.shape);
            ROS_INFO("==================================================");
        }
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}


