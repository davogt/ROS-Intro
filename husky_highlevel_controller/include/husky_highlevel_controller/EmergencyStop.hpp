/*
 * EmergencyStop.hpp
 *
 *  Created on: Mar 2, 2017
 *      Author: David Vogt
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Imu.h>

#include <std_srvs/SetBool.h>

namespace emergency_stop {

class EmergencyStop {
 public:
  void callService();

  void callback(const std_msgs::Float64ConstPtr& msg);
  void callback(const sensor_msgs::ImuConstPtr& msg);

  int loadParams();

  void enable();
  // Constructor:
  EmergencyStop(ros::NodeHandle& nodeHandle);
  // Destructor:
  virtual ~EmergencyStop();

 protected:
  ros::Subscriber subscriber_;
  ros::ServiceClient emergency_client;

 private:
  ros::NodeHandle nodeHandle_;

  std::string trigger_topic_name;
  bool hazard_detect = true;
  bool robot_state = false;

  double clearance = 1;

  std_srvs::SetBool service;
};

}

