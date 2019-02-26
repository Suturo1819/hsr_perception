/**
 * Main module for perception
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include "include/PerceptionActionServer.h"

int main(int argc, char** argv) {
   ros::init(argc, argv, "hsr_perception");

   PerceptionActionServer server("hsr_perception");
   ros::spin();

   return 0;

}