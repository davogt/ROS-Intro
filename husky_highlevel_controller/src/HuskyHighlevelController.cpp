#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <std_srvs/SetBool.h>

#include <unistd.h>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!loadParams()) {
    ROS_ERROR("Could not find topic/name parameter!");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe("/" + scan_topic_name, 3,
                                      &HuskyHighlevelController::callback,
                                      this);
  emergency_stop = nodeHandle.advertiseService("EmergencyStop", &HuskyHighlevelController::stopRobot, this);
  publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  vis_pub = nodeHandle.advertise<visualization_msgs::Marker>(
      "/visualization_marker", 10);
}

void HuskyHighlevelController::callback(
    const sensor_msgs::LaserScanConstPtr& msg) {

  const int size = (msg->ranges.size());
  float range[size];
  for (int i = 0; i < size; i++) {
    range[i] = msg->ranges[i];
  }
  float min = msg->range_min;
  float max = msg->range_max;
  float angle_min = msg->angle_min;
  float angle_max = msg->angle_max;
  float increment = msg->angle_increment;

  init_marker();

  ROS_INFO("Minimum measured distance is: %f", min_el(range, min, max, size));

  publisher_.publish(
      get_pos(range, min, max, size, angle_min, angle_max, increment));

  vis_pub.publish(marker);
}

float HuskyHighlevelController::min_el(float array[], float min, float max,
                                       int size) {
  float min_el = array[0];
  for (int i = 1; i < size; i++) {
    if ((min < array[i]) && (max > array[i])) {
      if (array[i] < min_el) {
        min_el = array[i];
      }
    }
  }
  return min_el;
}

geometry_msgs::Twist HuskyHighlevelController::get_pos(float array[], float min,
                                                       float max, int size,
                                                       float angle_min,
                                                       float angle_max,
                                                       float incr) {
  float min_el = array[0];

  float angle;
  float x = 0.;
  float y = 0.;

  int count = 0;

  float P = 1;  // Gain for P controller

  for (int i = 1; i < size; i++) {
    if ((min < array[i]) && (max > array[i])) {
      if (array[i] < min_el) {
        min_el = array[i];
        count = i;
      }
    }
  }
  angle = angle_min + incr * count;

  x = min_el * cos(angle);
  y = min_el * sin(angle);

  marker.pose.position.x = x;
  marker.pose.position.y = y;

  if (!stop_robot) {
    cmd_vel.linear.x = 2;
    cmd_vel.angular.z = -P * (angle - 0);
  } else;

  return cmd_vel;
}

void HuskyHighlevelController::init_marker() {
  marker.header.frame_id = "base_laser";
  marker.header.stamp = ros::Time();
  marker.ns = "husky_highlevel_controller";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .4;
  marker.scale.y = .4;
  marker.scale.z = 2.5;
  marker.color.a = 1.0f;  // Don't forget to set the alpha!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
}

int HuskyHighlevelController::loadParams() {
  if (!nodeHandle_.getParam("topic/name", scan_topic_name)) {
    return false;
  }
  if (!nodeHandle_.getParam("topic/size", scan_queue_size)) {
    return false;
  }
  return true;
}

bool HuskyHighlevelController::stopRobot(std_srvs::SetBool::Request &req,
                                         std_srvs::SetBool::Response &res) {
  if (!req.data){
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;

  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;

  stop_robot = !req.data;
  robot_state = req.data;
  }
  else {
    robot_state = req.data;
    stop_robot = !req.data;
  }
  res.success = (robot_state == req.data);
  if (res.success){
    res.message = "Robot state changed successfully!";
  }
  else res.message = "Could not change robot state..";

  return true;
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

} /* namespace */

