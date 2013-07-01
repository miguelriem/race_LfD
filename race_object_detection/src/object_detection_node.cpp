#ifndef _RACE_OBJECT_DETECTION_NODE_CPP_
#define _RACE_OBJECT_DETECTION_NODE_CPP_

#ifndef PFLN
#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);
#endif

#include <ros/ros.h>
#include <race_object_detection/race_object_detection.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "race_object_detection_node"); // Initialize ROS coms

	ros::NodeHandle n("~"); //The node handle
	race_object_detection::RaceObjectDetection<pcl17::PointXYZRGB> object_detection(&n);
	ros::spin();
}

#endif
