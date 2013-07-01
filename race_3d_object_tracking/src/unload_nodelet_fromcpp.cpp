/**
 * @file
 * @brief A simple example of how to unload a c_race_3d_object_tracking nodelet from cpp code.
 */
#ifndef _UNLOAD_NODELET_FROMCPP_CPP_
#define _UNLOAD_NODELET_FROMCPP_CPP_

#include <ros/ros.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tracker"); // Initialize ROS coms
	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle
	ros::Time t = ros::Time::now();
	ros::spinOnce();

	race_3d_object_tracking::unload_nodelet("/perception/tracking/", "/perception/nodelet_manager", n,
			"tracker1"); 

	return 1;
}

#endif
