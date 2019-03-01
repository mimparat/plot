#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <hanp_msgs/TrackedHumans.h>
#include <tf/transform_listener.h>
#include <std_msgs/Time.h>
#include <plot/StampedFloat.h>
#include <cmath>
#include "std_msgs/Header.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

class Subscriber{
public:
	Subscriber();
	void run();
	void topicCallback(const std_msgs::Float64::ConstPtr&  msg   );

private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub;
};

#endif SUBSCRIBER_H_
