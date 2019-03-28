/**
 * Action Server for table pipeline
 * @author: Fenja Kollasch
 */
#include <PerceptionActionServer.h>


PerceiveTable::PerceiveTable(std::string name, std::string savePath) :
        PerceptionActionServer(name, "hsrb_table", savePath),
        server(nh, name, boost::bind(&PerceiveTable::execute, this, _1), false)
{
    server.start();
    ROS_INFO("Perception Server for table observation started.");
}


void PerceiveTable::execute(const PerceiveTableGoalConstPtr &goal) {
    pm.run(goal->visualisation, result.detectionData);
    if(!result.detectionData.empty()) {
        feedback.feedback = "Object Feature detection was successful.";
        server.publishFeedback(feedback);
        server.setSucceeded(result);
    } else {
        feedback.feedback = "No object detection data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
    }
}
