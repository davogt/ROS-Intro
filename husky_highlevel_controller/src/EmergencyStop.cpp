/*
 * EmergencyStop.cpp
 *
 *  Created on: Mar 2, 2017
 *      Author: David Vogt
 */

#include <exception>
#include <iostream>
#include <ros/ros.h>
#include "husky_highlevel_controller/EmergencyStop.hpp"

#include <std_msgs/Float64.h>

#include <sensor_msgs/Imu.h>

#include <std_srvs/SetBool.h>

namespace emergency_stop {

EmergencyStop::EmergencyStop(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!loadParams()) {
    ROS_ERROR("Could not find trigger parameter!");
    ros::requestShutdown();
  }

  ros::ServiceClient emergency_client = nodeHandle
        .serviceClient<std_srvs::SetBool>(
        "/husky_highlevel_controller/EmergencyStop");


  try {
    if (trigger_topic_name == "imu/data")
    {
      subscriber_ = nodeHandle_.subscribe<const sensor_msgs::ImuConstPtr&>("/" + trigger_topic_name, 3,
                                        &EmergencyStop::callback, this);
    }
    else if (trigger_topic_name == "husky_highlevel_controller/min_dst"){
      subscriber_ = nodeHandle_.subscribe<const std_msgs::Float64ConstPtr&>("/" + trigger_topic_name, 3,
                                              &EmergencyStop::callback, this);
    }
    else ROS_WARN("No valid hazard avoidance have been set. Robot will not move..");

  } catch (...) {
    ROS_ERROR(
        "Could not resolve ROS trigger topic! Msg type is not valid to trigger Emergency Stop.");
  }
}

int EmergencyStop::loadParams() {
  if (!nodeHandle_.getParam("trigger_type", trigger_topic_name)) {
    return false;
  }
  return true;
}

void EmergencyStop::callService() {
service.request.data = hazard_detect;
  if (emergency_client.call(service)) {
    ROS_INFO("A hazard for the Robot was detected! Stopping all actuators..");
  }
  else {
    ROS_ERROR("Failed to call service \"Emergency Stop\"!");
  }
}

void EmergencyStop::callback(const std_msgs::Float64ConstPtr& msg) {
  if (msg->data < clearance){
    hazard_detect = true;

  }
  else hazard_detect = false;

  callService();
}

void EmergencyStop::callback(const sensor_msgs::ImuConstPtr& msg){

 }

EmergencyStop::~EmergencyStop() {
}

} /* namespace */
