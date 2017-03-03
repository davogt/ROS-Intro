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

#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>

#include <std_srvs/SetBool.h>

namespace emergency_stop {

EmergencyStop::EmergencyStop(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!loadParams()) {
    ROS_ERROR("Could not find trigger parameter!");
    ros::requestShutdown();
  }
  std::cout << trigger_topic_name;
  emergency_client = nodeHandle.serviceClient<std_srvs::SetBool>(
        "/husky_highlevel_controller/EmergencyStop");

  try {
    if (trigger_topic_name == "imu/data")
    {
      subscriber_ = nodeHandle_.subscribe<const sensor_msgs::ImuConstPtr&>("/" + trigger_topic_name, 3,
                                        &EmergencyStop::callback, this);
    }
    else if (trigger_topic_name == "husky_highlevel_controller/min_dst"){
      subscriber_ = nodeHandle_.subscribe<const std_msgs::Float32ConstPtr&>("/" + trigger_topic_name, 3,
                                              &EmergencyStop::callback, this);
    }
    else ROS_WARN("No valid hazard avoidance have been set. Robot will not move..");

  } catch (...) {
    ROS_ERROR(
        "Could not resolve ROS trigger topic! Msg type is not valid to trigger Emergency Stop.");
  }

  enable();
}

int EmergencyStop::loadParams() {
  if (!nodeHandle_.getParam("trigger_type", trigger_topic_name)) {
    return false;
  }
  return true;
}

void EmergencyStop::callService() {
service.request.data = !hazard_detect;
  if (emergency_client.call(service)) {
    if (hazard_detect){
      ROS_INFO("A hazard for the Robot was detected! Stopping all actuators..");
    }
    else ROS_INFO("I'll be roving on");
  }
  else {
    ROS_ERROR("Failed to call service \"Emergency Stop\"!");
  }
}

void EmergencyStop::enable(){
  hazard_detect = false;
  callService();
  ROS_INFO("Robot has been enabled. start driving..");
}

void EmergencyStop::callback(const std_msgs::Float32ConstPtr& msg) {
  if (msg->data < clearance){
    hazard_detect = true;
  }
  else hazard_detect = false;

  callService();
}

void EmergencyStop::callback(const sensor_msgs::ImuConstPtr& msg){
  if (msg->linear_acceleration.z > 15.0){
    hazard_detect = true;
    callService();
  }
  else hazard_detect = false;
 }

EmergencyStop::~EmergencyStop() {
}

} /* namespace */
