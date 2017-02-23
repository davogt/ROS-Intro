#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
  void callback(const sensor_msgs::LaserScanConstPtr& msg);
  float min_el(float array[], float min, float max, int size);
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle, std::string topic_name, int queue_size);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

protected:
ros::Subscriber subscriber_;

private:
	ros::NodeHandle nodeHandle_;
  std::string objectName="HuskyHighlevelController";

  std::string scan_topic_name;
  int scan_queue_size;
};

} /* namespace */
