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

// RoboSherlock stuff
#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/scene_cas.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <mongo/client/init.h>

using namespace suturo_perception_msgs;

class PerceptionActionServer {

protected:
    ros::NodeHandle nh;
    std::string action_name;
    RSAnalysisEngine engine;

    void process(std::vector<ObjectDetectionData> &detection_data);

    void getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data);

public:
    PerceptionActionServer(std::string &name, std::string pipeline);

    ~PerceptionActionServer(){}
};

class PerceiveTable : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<PerceiveTableAction> server;
    PerceiveTableFeedback feedback;
    PerceiveTableResult result;

public:
    PerceiveTable(std::string name);
    void execute(const PerceiveTableGoalConstPtr &goal);
};

class PerceiveShelf : PerceptionActionServer {
protected:
    actionlib::SimpleActionServer<PerceiveShelfAction> server;
    PerceiveShelfFeedback feedback;
    PerceiveShelfResult result;

public:
    PerceiveShelf(std::string name);
    void execute(const PerceiveShelfGoalConstPtr &goal);
};
