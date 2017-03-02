#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <std_srvs/SetBool.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
 public:
  // Callback function that takes runs the code if a message of the scan topic is in the buffer
  void callback(const sensor_msgs::LaserScanConstPtr& msg);
  // Stop Function sets all linear/angular velocities to zero and returns the robot state.
  bool stopRobot(std_srvs::SetBool::Request &req,
                 std_srvs::SetBool::Response &res);
  // Functions used within callback to determine the distance to the closest object
  // and its relative coordinates
  float min_el(float array[], float min, float max, int size);
  geometry_msgs::Twist get_pos(float array[], float min, float max, int size,
                               float angle_min, float angle_max, float incr);
  void init_marker();
  // Constructor:
  HuskyHighlevelController(ros::NodeHandle& nodeHandle);
  // Destructor:
  virtual ~HuskyHighlevelController();

 protected:
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  ros::Publisher vis_pub;
  ros::ServiceServer emergency_stop;

 private:

  ros::NodeHandle nodeHandle_;
  std::string objectName = "HuskyHighlevelController";

  int loadParams();
  std::string scan_topic_name;
  int scan_queue_size;

  bool stop_robot = true;        // Stops the robot if == true
  bool robot_state = false;  // Returns true if robot is moving, false if all velocities == 0
  geometry_msgs::Twist cmd_vel;
  visualization_msgs::Marker marker;
};

} /* namespace */
