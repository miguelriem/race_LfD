#ifndef _RACE_3D_OBJECT_TRACKING_EXAMPLE_CPP_
#define _RACE_3D_OBJECT_TRACKING_EXAMPLE_CPP_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);
#include <ros/ros.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>

using namespace race_3d_object_tracking;

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


int main (int argc, char** argv)
{
	ros::init(argc, argv, "particle_filter_tracking"); // Initialize ROS coms
	ros::NodeHandle* n;
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	c_race_3d_object_tracking<pcl17::PointXYZRGBA> tracker(n);

	ros::Time t = ros::Time::now();
	ros::Rate loop_rate(10);
	ros::spin();

	return 1;
}

