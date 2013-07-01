/**
 * @file
 * @brief A simple example of how to launch a c_race_3d_object_tracking nodelet from cpp code.
 */
#ifndef _LOAD_NODELET_FROMCPP_CPP_
#define _LOAD_NODELET_FROMCPP_CPP_

#include <ros/ros.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tracker"); // Initialize ROS coms
	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle
	ros::Time t = ros::Time::now();
	ros::spinOnce();


	race_3d_object_tracking::load_nodelet("/perception/tracking/", "/perception/nodelet_manager", n,
			"tracker1", 
			"/home/mike/workingcopy/race_LfD/race_3d_object_tracking/pcd/obj1.pcd", 
			"/camera_rgb_optical_frame", 
			"/camera/depth_registered/points", 
			0.8, 400, 0.2, 0.1, 0.0, 0.0, 1.0, 0.1, 0., 0., 1., 1.); 

	return 1;
}

#endif
