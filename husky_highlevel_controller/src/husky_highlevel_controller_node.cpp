#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}
