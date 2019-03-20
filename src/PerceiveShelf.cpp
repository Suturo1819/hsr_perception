/**
 * Action Server for table pipeline
 * @author: Fenja Kollasch
 */
#include <PerceptionActionServer.h>


PerceiveShelf::PerceiveShelf(std::string name) :
PerceptionActionServer(name, "hsrb_shelf"),
server(nh, name, boost::bind(&PerceiveShelf::execute, this, _1), false)
{
server.start();
ROS_INFO("Perception Server for table observation started.");
}

void PerceiveShelf::execute(const PerceiveShelfGoalConstPtr &goal) {
    process(result.detectionData);
    if(!result.detectionData.empty()) {
        feedback.feedback = "Object Feature detection was successful.";
        server.publishFeedback(feedback);
        result.inliers_visible = 1;
        server.setSucceeded(result);
    } else {
        result.inliers_visible = 0;
        feedback.feedback = "No object detection data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
        server.setSucceeded(result);
    }
}

