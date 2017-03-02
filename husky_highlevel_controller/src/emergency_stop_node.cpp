/*
 * emergency_stop_node.cpp
 *
 *  Created on: Mar 2, 2017
 *      Author: David Vogt
 */

#include <ros/ros.h>
#include "husky_highlevel_controller/EmergencyStop.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stop");
  ros::NodeHandle nodeHandle("~");

  emergency_stop::EmergencyStop emergencyObject(nodeHandle);

  ros::spin();
  return 0;
}
