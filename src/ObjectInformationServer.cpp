/**
 * Runs the RoboSherlock Pipeline and collects information about each recognized object
 * The information is encapsuled as an ObjectDetectionData object
 * @author: Fenja Kollasch
 */

#include <PerceptionActionServer.h>


ObjectInformationServer::ObjectInformationServer(std::string name, std::string savePath) :
        PerceptionActionServer(name, "suturo_pipeline", savePath),
        server(nh, name, boost::bind(&ObjectInformationServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("Object Information Server started.");
}


void ObjectInformationServer::execute(const ExtractObjectInfoGoalConstPtr &goal) {
    std::map<std::string, boost::any> arguments = std::map<std::string, boost::any>();
    arguments["visualize"] = goal->visualize;
    std::vector<std::string> regions = std::vector<std::string>();
    arguments["regions"] = goal->regions;
    pm.run(arguments, result.detectionData);
    if(!result.detectionData.empty()) {
        feedback.feedback = "Object Feature detection was successful.";
        server.publishFeedback(feedback);
        server.setSucceeded(result);
    } else {
        feedback.feedback = "No object detection data was perceived. Check standard output for further information.";
        server.publishFeedback(feedback);
    }
}

