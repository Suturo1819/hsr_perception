/**
 * Action Server for table pipeline
 * @author: Fenja Kollasch
 */
#include <PerceptionActionServer.h>


PerceiveShelf::PerceiveShelf(std::string name, std::string savePath) :
PerceptionActionServer(name, "hsrb_shelf", savePath),
server(nh, name, boost::bind(&PerceiveShelf::execute, this, _1), false)
{
server.start();
ROS_INFO("Perception Server for shelf observation started.");
}

void PerceiveShelf::execute(const PerceiveShelfGoalConstPtr &goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualisation;
    std::vector<std::string> regions = std::vector<std::string>();
    regions.emplace_back("robocup_shelf_2");
    arguments["regions"] = regions;
    pm.run(arguments, result.detectionData);
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

