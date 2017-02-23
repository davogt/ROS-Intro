#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


/*void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float min_dst = msg->ranges[0];
  int size = ((sizeof msg->ranges)/(sizeof msg->ranges[0]));

  for (int i = 1; i < size; i++)
  {
    if ((msg->range_min < msg->ranges[i]) & (msg->range_max > msg->ranges[i]))
    {
      if (msg->ranges[i] < min_dst)
      {
        min_dst = msg->ranges[i];
      }
    }
  }
  ROS_INFO("The minimum distance is: %f \n", min_dst);

}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");
  ros::NodeHandle node;
  std::string topic_name;
  int queue_size;

  if (!nodeHandle.getParam("topic/name", topic_name)) {
    ROS_ERROR("Could not find topic/name parameter!");
  }
  if (!nodeHandle.getParam("topic/size", queue_size)) {
    ROS_ERROR("Could not find topic/size parameter!");
  }


//  ros::Subscriber subscriber = nodeHandle.subscribe("/" + topic_name, queue_size, chatterCallback);

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle, topic_name, queue_size);

  ros::spin();
  return 0;
}
