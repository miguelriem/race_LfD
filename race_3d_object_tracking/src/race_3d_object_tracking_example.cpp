#ifndef _RACE_3D_OBJECT_TRACKING_EXAMPLE_CPP_
#define _RACE_3D_OBJECT_TRACKING_EXAMPLE_CPP_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);
#include <ros/ros.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


c_race_3d_object_tracking<pcl17::PointXYZRGBA>* p_objt;
bool received_obj1=false;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_obj_to_track;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_objects_on_table;
pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_aligned;
ros::Publisher* p_pub_pc1;
ros::Publisher* p_pub_pc2;
ros::NodeHandle* n;

//boost::shared_ptr<pcl17::tracking::ParticleFilterTracker<pcl17::PointXYZRGBA, pcl17::tracking::ParticleXYZRPY> > tracker_;


//ros::Publisher* p_marker_publisher;
boost::shared_ptr<tf::TransformBroadcaster> br;

boost::shared_ptr<visualization_msgs::MarkerArray> marker_array; //global variable with the marker array.

void callback_obj1_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
{
	if (received_obj1==false)
	{
		ROS_INFO("Received obj1 Point Cloud. Processing callback."); 
		pcl17::fromROSMsg(*pcmsg_in, *cloud_obj_to_track);

		p_objt->set_obj_to_track(cloud_obj_to_track);
		//Eigen::Vector4f c;
		//pcl17::compute3DCentroid<pcl17::PointXYZRGBA> (*cloud_obj_to_track, c);
		//Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
		//trans.translation() = Eigen::Vector3f (c[0], c[1], c[2]);

		//pcl17::transformPointCloud<pcl17::PointXYZRGBA> (*cloud_obj_to_track, *cloud_obj_to_track, trans.inverse());

		//tracker_->setTrans (trans);
		////reference_ = transed_ref;
		//tracker_->setMinIndices (int (cloud_obj_to_track->points.size ()) / 2);

		//tracker_->setReferenceCloud(cloud_obj_to_track);
		received_obj1 = true;
	}
}

void callback_objects_on_table_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
{
	if (received_obj1==true)
	{
		ROS_INFO("Received objects on table. Commencing ICP registration."); 
		pcl17::fromROSMsg(*pcmsg_in, *cloud_objects_on_table);

		p_objt->track_obj(cloud_objects_on_table);

		//sensor_msgs::PointCloud2 msg; 
		//pcl::toROSMsg(*cloud_aligned,msg); 	
		//p_pub_pc1->publish(msg);

		//pcl::toROSMsg(*cloud_obj_to_track,msg); 	
		//p_pub_pc2->publish(msg);

		//cloud_obj_to_track = cloud_aligned;

		//double start = pcl17::getTime ();
		//FPS_CALC_BEGIN;
		//ROS_INFO("size cloud_objects_on_table = %ld",cloud_objects_on_table->points.size());
		//tracker_->setInputCloud (cloud_objects_on_table);
		//tracker_->compute ();
		////double end = pcl17::getTime ();
		////FPS_CALC_END("tracking");
		////tracking_time_ = end - start;

		//tracker_->setReferenceCloud(cloud_obj_to_track);


		////draw particles
		//pcl17::tracking::ParticleFilterTracker<pcl17::PointXYZRGBA, pcl17::tracking::ParticleXYZRPY>::PointCloudStatePtr particles = 
			//tracker_->getParticles ();

		//if (particles)
		//{
			//visualization_msgs::Marker marker;
			//marker.header.frame_id = "/openni_rgb_optical_frame";
			//marker.header.stamp = ros::Time();
			//std::stringstream sstm;

			////prepare points marker
			//marker.ns = "particles";
			//marker.id = 0;
			//marker.type = visualization_msgs::Marker::POINTS;
			//marker.action = visualization_msgs::Marker::ADD;
			//marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
			//marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
			//marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; 

			//marker.color.r =0; 
			//marker.color.g =0; 
			//marker.color.b =1; 
			//marker.color.a =1; 
			////= cm->color(i);
			////marker.lifetime = ros::Duration(1); //one sec lifetime

			//marker.points.erase(marker.points.begin(), marker.points.end());
			//geometry_msgs::Point p;

			//ROS_WARN("There are %ld particles in the filter",particles->points.size ());
			//for (size_t i = 0; i < particles->points.size (); i++)
			//{
				//p.x = particles->points[i].x;
				//p.y = particles->points[i].y;
				//p.z = particles->points[i].z;
				//marker.points.push_back(p);
			//}

			//marker_array.markers.push_back(marker);

		//}
		//else
		//{
			//ROS_WARN("There are no particles in the filter");
		//}


		//pcl17::tracking::ParticleXYZRPY result = tracker_->getResult ();
		//Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
		//// move a little bit for better visualization
		////transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
		////RefCloudPtr result_cloud (new RefCloud ());

		//pcl17::transformPointCloud<pcl17::PointXYZRGBA>(*(tracker_->getReferenceCloud()), *cloud_aligned, transformation);

		p_objt->set_visualization_marker_array();

		sensor_msgs::PointCloud2 msg; 
		pcl17::toROSMsg(*p_objt->get_cloud_aligned(),msg); 	
		p_pub_pc1->publish(msg);



	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "particle_filter_tracking"); // Initialize ROS coms
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	cloud_obj_to_track = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	cloud_objects_on_table = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	cloud_aligned = (pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr) new pcl17::PointCloud<pcl17::PointXYZRGBA>;
	marker_array = (boost::shared_ptr<visualization_msgs::MarkerArray>) new visualization_msgs::MarkerArray;


	br = (boost::shared_ptr<tf::TransformBroadcaster>) new tf::TransformBroadcaster;


	ros::Subscriber sub_pc = n->subscribe ("/pc_out/obj1", 1, callback_obj1_received);
	ros::Subscriber sub_pc1 = n->subscribe ("/pc_out/objects_on_table", 1, callback_objects_on_table_received);

	ros::Publisher pub_pc1 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/aligned", 1); p_pub_pc1 = &pub_pc1;
	ros::Publisher pub_pc2 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/object_to_track", 1); p_pub_pc2 = &pub_pc2;

	//setup marker publishers
	//ros::Publisher marker_publisher = n->advertise<visualization_msgs::MarkerArray>("/PF", 1); p_marker_publisher = &marker_publisher;

	std_msgs::ColorRGBA color;
	color.a=1;
	color.b=1;
	c_race_3d_object_tracking<pcl17::PointXYZRGBA> tracker(br, "openni_rgb_optical_frame",n, "obj1", 0.99, 300, 0.2, 0.1, color);
	p_objt = &tracker;

	ros::Time t = ros::Time::now();
	//Start program
	ros::Rate loop_rate(10);
	ros::spin();


	return 1;
}

