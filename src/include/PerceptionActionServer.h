/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/PerceiveTableAction.h>
#include <suturo_perception_msgs/PerceiveShelfAction.h>

#include <rs_hsrb_perception/SuturoProcessManager.h>

using namespace suturo_perception_msgs;

class PerceptionActionServer {

protected:
    ros::NodeHandle nh;
    std::string action_name;
    SuturoProcessManager pm;


public:
    PerceptionActionServer(std::string &name, std::string pipeline, std::string savePath);

    ~PerceptionActionServer(){}
};

class PerceiveTable : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<PerceiveTableAction> server;
    PerceiveTableFeedback feedback;
    PerceiveTableResult result;

public:
    PerceiveTable(std::string name, std::string savePath);
    void execute(const PerceiveTableGoalConstPtr &goal);
};

class PerceiveShelf : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<PerceiveShelfAction> server;
    PerceiveShelfFeedback feedback;
    PerceiveShelfResult result;

public:
    PerceiveShelf(std::string name, std::string savePath);
    void execute(const PerceiveShelfGoalConstPtr &goal);
};
