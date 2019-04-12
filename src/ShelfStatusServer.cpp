/**
 * Creates a RoboSherlock pipeline optimized to detect whether the shelf door is closed or not
 * @author: Fenja Kollasch
 */
#include <PerceptionActionServer.h>
ShelfStatusServer::ShelfStatusServer(std::string name, std::string savePath) :
        PerceptionActionServer(name, "shelf_door", savePath),
        server(nh, name, boost::bind(&ShelfStatusServer::execute, this, _1), false)
{
    server.start();
    ROS_INFO("Shelf Status Server started.");
}

void ShelfStatusServer::execute(const AnalyzeShelfStatusGoalConstPtr &goal) {
    bool door = pm.has_vertical_plane();
    if(door) {
        result.door_open = 0;
        feedback.feedback = "Shelf door is shut";
    } else {
        result.door_open = 1;
        feedback.feedback = "Shelf door is open";
    }
    server.publishFeedback(feedback);
    server.setSucceeded(result);
}
