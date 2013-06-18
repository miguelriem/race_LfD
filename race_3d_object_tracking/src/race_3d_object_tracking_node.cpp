#ifndef _RACE_3D_OBJECT_TRACKING_NODE_CPP_
#define _RACE_3D_OBJECT_TRACKING_NODE_CPP_

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);
#include <ros/ros.h>

#include <pc_processing/pc_processing.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>

#include <race_3d_object_tracking/tracker_info.h>
/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


c_race_3d_object_tracking<pcl17::PointXYZRGBA>* p_objt;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_obj_to_track;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_in;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_in_downsampled;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_aligned;
ros::Publisher* p_pub_pc1;
ros::Publisher* p_pub_tracker_info;
ros::NodeHandle* n;
ros::Time StartTic;
//std::string node_name;

//ros::Publisher* p_marker_publisher;
boost::shared_ptr<tf::TransformBroadcaster> br;

//boost::shared_ptr<visualization_msgs::MarkerArray> marker_array; //global variable with the marker array.

bool publish_tracker_info(bool object_lost)
{
	//ROS_INFO("%s: Pulishing tracker_info msg", ros::this_node::getName().c_str());

	race_3d_object_tracking::tracker_info ti;
	ti.bb_dx = p_objt->bb_dx;
	ti.bb_dy = p_objt->bb_dy;
	ti.bb_dz = p_objt->bb_dz;
	ti.object_frame_id = p_objt->my_tf;
	ti.parent_frame_id = p_objt->parent_tf;
	ti.time_since_start = ros::Time::now() - StartTic;
	ti.object_lost = object_lost;

	p_pub_tracker_info->publish(ti);

	return true;
}


void pc_in_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
{
	//ROS_INFO("%s: Received pc_in. Update tracking", ros::this_node::getName().c_str()); 

	pcl17::fromROSMsg(*pcmsg_in, *cloud_in);

	//Downsample input point cloud 
	if (!downsample_pc(cloud_in, 0.01, 0.01, 0.01,	cloud_in_downsampled))                                                                         
		ROS_ERROR("Could not downsample point cloud");

	//update the tracking
	cloud_in_downsampled->header.stamp = pcmsg_in->header.stamp;
	p_objt->track_obj(cloud_in_downsampled);

	//ROS_INFO("%s: fit ratio = %f",ros::this_node::getName().c_str(), p_objt->t->getFitRatio());

	bool flg_keep_tracking=true;
	if (p_objt->t->getFitRatio() > -50.0 )
	{
	
		ROS_WARN("%s: fit ratio = %f. Bellow threshold. Tracking lost object. Shuting down node.",ros::this_node::getName().c_str(), p_objt->t->getFitRatio());
		flg_keep_tracking=false;
	}

	publish_tracker_info(!flg_keep_tracking);


	p_objt->set_visualization_marker_array();

	sensor_msgs::PointCloud2 msg; 
	pcl17::toROSMsg(*p_objt->get_cloud_aligned(),msg); 	
	p_pub_pc1->publish(msg);

	if (flg_keep_tracking==false)
	{
		ros::Duration(1.0).sleep();
		ros::shutdown();
	}
}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "tracker"); // Initialize ROS coms

	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	ros::Time t = ros::Time::now();
	ros::spinOnce();

	std::string parent_frame_name="not assigned";
	n->getParam("parent_frame_id", parent_frame_name);
	ROS_INFO("parent_frame_id remaped to %s", parent_frame_name.c_str());

	//Set the color for rviz msgs
	std_msgs::ColorRGBA color;
	double tmp;
	n->getParam("color_r", tmp); color.r = tmp;
	n->getParam("color_g", tmp); color.g = tmp;
	n->getParam("color_b", tmp); color.b = tmp;
	n->getParam("color_a", tmp); color.a = tmp;

	//Get tracking parameters
	double delta = 0.99; n->getParam("delta", delta); 
	int maxparticlenum = 300; n->getParam("maxparticlenum", maxparticlenum); 
	double epsilon = 0.2; n->getParam("epsilon", epsilon);
	double bin = 0.1; n->getParam("bin", bin);

	//initialize point clouds
	cloud_obj_to_track = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	cloud_in = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	cloud_in_downsampled = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	cloud_aligned = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	//marker_array = (boost::shared_ptr<visualization_msgs::MarkerArray>) new visualization_msgs::MarkerArray;

	//initialize transform broadcaster
	br = (boost::shared_ptr<tf::TransformBroadcaster>) new tf::TransformBroadcaster;

	//subscribe to point cloud messages to update tracking
	std::string pc_in_name; n->getParam("pc_in", pc_in_name); 
	//ROS_INFO("pc_in = %s",pc_in_name.c_str());
	ros::Subscriber sub_pc1 = n->subscribe (pc_in_name, 1, pc_in_received);

	ros::Publisher pub_pc1 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/aligned", 1); p_pub_pc1 = &pub_pc1;

	//setup marker publishers
	//ros::Publisher marker_publisher = n->advertise<visualization_msgs::MarkerArray>("/PF", 1); p_marker_publisher = &marker_publisher;


	std::string pcd_file_path;
	n->getParam("object_pcd_file", pcd_file_path);
	if (pcl17::io::loadPCDFile<pcl17::PointXYZRGBA> (pcd_file_path.c_str(), *cloud_obj_to_track) == -1) //* load the file
	{
		ROS_ERROR("%s: Couldn't read file %s\n", ros::this_node::getName().c_str(), pcd_file_path.c_str());
		return (-1);
	}
	cloud_obj_to_track->header.frame_id = parent_frame_name.c_str();

	ROS_INFO("parent_frame is %s",parent_frame_name.c_str());

	//initialize tracker class
	c_race_3d_object_tracking<pcl17::PointXYZRGBA> tracker(br, parent_frame_name.c_str(), n,ros::this_node::getName() , delta, maxparticlenum, epsilon, bin, color);
	p_objt = &tracker;

	//set object to track point cloud
	p_objt->set_obj_to_track(cloud_obj_to_track);


	ROS_INFO("%s: Tracker initialized with the following parameters:\ndelta = %f\nmaxparticlenum = %d\nepsilon = %f\nbin = %f\nColor [rgba] = [%f,%f,%f,%f]\nobject point cloud loaded from %s contains %ld points\nsubscribed to %s for tracking",ros::this_node::getName().c_str(), delta,maxparticlenum, epsilon, bin,  color.r, color.g, color.b, color.a, pcd_file_path.c_str(), cloud_obj_to_track->points.size(), pc_in_name.c_str());

	//Setup the tracked_obj_info service
	std::string tracker_info_name = ros::this_node::getName() + "/info";   

	ros::Publisher pub_tracker_info = n->advertise<race_3d_object_tracking::tracker_info>(tracker_info_name, 1); 
	p_pub_tracker_info = &pub_tracker_info;
	ROS_INFO("%s: Publisher %s initialized",ros::this_node::getName().c_str(), tracker_info_name.c_str());

	StartTic=ros::Time::now();

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
