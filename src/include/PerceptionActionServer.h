/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/PerceiveAction.h>

// RoboSherlock stuff
#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/scene_cas.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>

using namespace suturo_perception_msgs;

class PerceptionActionServer {

protected:
    ros::NodeHandle nh;
    std::string action_name;
    actionlib::SimpleActionServer<PerceiveAction> server;

    PerceiveFeedback feedback;
    PerceiveResult result;
    RSAnalysisEngine engine;

    void execPipeline(std::string pipeline);
    void getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data);

public:
    PerceptionActionServer(std::string name) :
            action_name(name),
            server(nh, name, boost::bind(&PerceptionActionServer::execute, this, _1), false)
    {
        server.start();
        ROS_INFO("Perception Server started.");

    }

    ~PerceptionActionServer(){}

    void execute(const PerceiveGoalConstPtr &goal);
};
