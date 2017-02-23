#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <unistd.h>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle, std::string topic_name, int queue_size)
    : nodeHandle_(nodeHandle), scan_topic_name(topic_name), scan_queue_size(queue_size)
{
    subscriber_ = nodeHandle_.subscribe("/" + scan_topic_name, 3, &HuskyHighlevelController::callback, this);
}

void HuskyHighlevelController::callback(const sensor_msgs::LaserScanConstPtr& msg)
{

  const int size = ((sizeof msg->ranges)/(sizeof msg->ranges[0]));
  float range[size] = {0};
  for (int i = 0; i < size; i++){range[i] = msg->ranges[i];}
  float min = msg->range_min;
  float max = msg->range_max;
  sleep(1);
  //ROS_INFO("Callback from object: -- %s --", objectName.c_str());
  ROS_INFO("Minimum measured distance is: %f", min_el(range, min, max, size));
}

float HuskyHighlevelController::min_el(float array[], float min, float max, int size)
{
  float min_el = array[0];
  for (int i = 1; i < size; i++)
    {
      //if ((min < array[i]) && (max > array[i]))
      //{
        if (array[i] < min_el)
        {
          min_el = array[i];
        }
      //}
    }
  return min_el;
}


HuskyHighlevelController::~HuskyHighlevelController() {
}

} /* namespace */

