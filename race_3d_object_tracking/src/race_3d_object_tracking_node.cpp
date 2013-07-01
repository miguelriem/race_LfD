/**
 * @file
 * @brief A simple example of how to use c_race_3d_object_tracking from a node.
 */
#ifndef _RACE_3D_OBJECT_TRACKING_NODE_CPP_
#define _RACE_3D_OBJECT_TRACKING_NODE_CPP_

#include <ros/ros.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>
using namespace race_3d_object_tracking;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tracker"); // Initialize ROS coms

	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	ros::Time t = ros::Time::now();
	ros::spinOnce();

	//initialize tracker class
	c_race_3d_object_tracking<pcl17::PointXYZRGBA> tracker(n);

	//Start program
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		ros::spinOnce();                   // Handle ROS events
		loop_rate.sleep();
	}

	return 1;
}

#endif
