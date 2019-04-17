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
   ros::MultiThreadedSpinner spinner(4);
   ShelfStatusServer shelf_status_server("analyze_shelf_status", "/home/suturo/Desktop/temp");
   ObjectInformationServer object_info_server("extract_object_infos", "/home/suturo/Desktop/temp");
   spinner.spin();

   return 0;

}
