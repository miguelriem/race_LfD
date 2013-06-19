#ifndef _RACE_TABLETOP_SEGMENTATION_NODE_CPP_
#define _RACE_TABLETOP_SEGMENTATION_NODE_CPP_

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>
#include <race_tabletop_segmentation/race_tabletop_segmentation.h>


int main (int argc, char** argv)
{
	ros::init(argc, argv, "race_tabletop_segmentation_node"); // Initialize ROS coms

	ros::NodeHandle n; //The node handle

	race_tabletop_segmentor::RaceTabletopSegmentor<pcl17::PointXYZRGB> tabletop_segmentor(&n);

	//Start program
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		tabletop_object_detector::TabletopSegmentation srv;
		tabletop_object_detector::TabletopSegmentation::Request request;
		tabletop_object_detector::TabletopSegmentation::Response response;
		tabletop_segmentor.serviceCallback(request, response);
		ros::spinOnce();                   // Handle ROS events
		loop_rate.sleep();
	}

	return 1;

	}

#endif
