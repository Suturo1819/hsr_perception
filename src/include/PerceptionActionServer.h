/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/ExtractObjectInfoAction.h>
#include <suturo_perception_msgs/AnalyzeShelfStatusAction.h>

#include <rs_hsrb_perception/SuturoProcessManager.h>

using namespace suturo_perception_msgs;

class PerceptionActionServer {

protected:
    ros::NodeHandle nh;
    std::string action_name;
    SuturoProcessManager pm;


public:
    PerceptionActionServer(std::string &name, std::string pipeline, std::string savePath);

    ~PerceptionActionServer(){};
};

class ObjectInformationServer : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<ExtractObjectInfoAction> server;
    ExtractObjectInfoFeedback feedback;
    ExtractObjectInfoResult result;

public:
    ObjectInformationServer(std::string name, std::string savePath);
    void execute(const ExtractObjectInfoGoalConstPtr &goal);
};

class ShelfStatusServer : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<AnalyzeShelfStatusAction> server;
    AnalyzeShelfStatusFeedback feedback;
    AnalyzeShelfStatusResult result;

public:
    ShelfStatusServer(std::string name, std::string savePath);
    void execute(const AnalyzeShelfStatusGoalConstPtr &goal);
};
