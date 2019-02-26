/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/PerceiveAction.h>

using namespace suturo_perception_msgs;

class PerceptionActionServer {

protected:
    ros::NodeHandle nh;
    std::string action_name;
    actionlib::SimpleActionServer<PerceiveAction> server;

    PerceiveFeedback feedback;
    PerceiveResult result;

    void execTable() {

    }

    void execShelf() {

    }

public:
    PerceptionActionServer(std::string name) :
        action_name(name),
        server(nh, name, boost::bind(&PerceptionActionServer::execute, this, _1), false)
    {
        server.start();
    }

    ~PerceptionActionServer(){}

    void execute(const PerceiveGoalConstPtr &goal) {
        auto pipeline = goal->pipeline;
        if(pipeline == "table") {
            execTable();
        } else if(pipeline == "shelf") {
            execShelf();
        } else {
            feedback.feedback = "There is no pipeline defined for " + pipeline;
            server.publishFeedback(feedback);
        }
    }

};

