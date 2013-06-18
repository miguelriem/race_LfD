#ifndef _GESTURE_DETECTION_CPP_
#define _GESTURE_DETECTION_NODE_CPP_

#include "gesture_detection_node.h"

double sphere_radius = 0.2;

void callback_active_object_received(const race_learning_by_demonstration::active_object_list_msg& msg_in)
{

	/* _________________________________
	   |                                 |
	   |     VIRTUAL HAND EXTENSION      |
	   |_________________________________| */

	//Build a 3d line segment from hand in the direction of the elbow-hand vector
	//This is a point cloud with only 2 points. line_segment_local 
	pcl17::PointXYZ pt1, pt2;
	double pointing_size = 0.2; /*meters*/
	tf::StampedTransform transform;
	ros::Time t=ros::Time::now();

	bool flg_compute_VHE = true;
	//For the second point we need the direction, for that we query the hand to
	//elbow transformation

	try
	{
		p_tf_listener->lookupTransform("/left_elbow_1", "/left_hand_1", t, transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		flg_compute_VHE = false;
	}	  

	if (flg_compute_VHE)
	{
		double vector[3];
		vector[0] = transform.getOrigin().x();
		vector[1] = transform.getOrigin().y();
		vector[2] = transform.getOrigin().z();
		normalize_vector(vector);

		//The first pt is the origin of the hand tf
		pt1.x =	transform.getOrigin().x();
		pt1.y =	transform.getOrigin().y();
		pt1.z =	transform.getOrigin().z();

		pt2.x = pt1.x + pointing_size * vector[0];
		pt2.y = pt1.y + pointing_size * vector[1];
		pt2.z = pt1.z + pointing_size * vector[2];

		//Now we insert pt1 and pt2 to the line_segment_local point cloud
		line_segment_local->points.erase( line_segment_local->points.begin(), line_segment_local->points.end());
		line_segment_local->points.push_back(pt1);
		line_segment_local->points.push_back(pt2);
		line_segment_local->header.frame_id = "/left_elbow_1";

		//Finally, we compute the point cloud in the /openni_rgb_optical_frame for
		//later comparison with the sphere of the objects
		try
		{
			p_tf_listener->lookupTransform("/left_elbow_1", "/openni_rgb_optical_frame", t, transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}	  

		pcl17_ros::transformPointCloud(*line_segment_local, *line_segment_global,  transform.inverse());
		line_segment_global->header.frame_id = "/openni_rgb_optical_frame";

		/* _________________________________
		   |                                 |
		   | VHE INTERSECTIONS WITH SPHERES  |
		   |_________________________________| */

		for (size_t i = 0; i < msg_in.object.size(); i++) //cycle all active objects and test their intersection with the VHE
		{
			bool have_transform = true;

			if(p_tf_listener.waitForTransform("/openni_rgb_optical_frame", msg_in.object.at(i).frame_id, t, ros::Duration(1.0)))
			{   
				try 
				{   
					p_tf_listener->lookupTransform("/openni_rgb_optical_frame", msg_in.object.at(i).frame_id, t, transform);
				}   
				catch (tf::TransformException ex) 
				{   
					ROS_ERROR("Could not lookup transform:\n%s",ex.what());
					have_transform = false;
				}   
			}   
			else
			{   
				ROS_ERROR("Could find valid transform after waiting 1 secs\n");
				have_transform = false;
			}   

			if (have_transform==true)
			{
				bool is_pointed_to = does_line_segment_intersect_sphere( line_segment_global->points.at(0).x, 
					line_segment_global->points.at(0).y, 
					line_segment_global->points.at(0).z,
					line_segment_global->points.at(1).x, 
					line_segment_global->points.at(1).y, 
					line_segment_global->points.at(1).z,
					transform.getOrigin().x(), 
					transform.getOrigin().y(),
					transform.getOrigin().z(),
					sphere_radius);
			}
		}



	}
}


/**
 * @brief 
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gesture_detection"); // Initialize ROS coms
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	//Define subscribers
	ros::Subscriber sub_pc = n->subscribe("/LbD/active_object", 1, callback_active_object_received);

	//Initialize the transform listener
	tf::TransformListener tf_listener; p_tf_listener = &tf_listener;

	//ros::Publisher marker_publisher = n->advertise<visualization_msgs::MarkerArray>("/LBD", 1); p_marker_publisher = &marker_publisher;
	//ros::Publisher marker_publisher1 = n.advertise<visualization_msgs::MarkerArray>("/PF", 1); p_marker_publisher1 = &marker_publisher1;




	//initialize the hand pointing point clouds
	line_segment_local = (boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >)new pcl17::PointCloud<pcl17::PointXYZ>;
	line_segment_global = (boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >)new pcl17::PointCloud<pcl17::PointXYZ>;
	ros::Time t = ros::Time::now();

	//Start program
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();                   // Handle ROS events
		loop_rate.sleep();
	}
	return 1;
}

#endif
