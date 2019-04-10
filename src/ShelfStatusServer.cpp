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
    //@Todo: figure out what to do here...
}
