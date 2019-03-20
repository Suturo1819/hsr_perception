/**
 * Main module for perception
 * Creates Action servers for all perception pipelines
 * Initializes RoboSherlock
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <PerceptionActionServer.h>

int main(int argc, char** argv) {
   ros::init(argc, argv, "hsr_perception");

   PerceiveTable table_server("hsr_perception_table");
   PerceiveShelf shelf_server("hsr_perception_shelf");
   ros::spin();

   return 0;

}