#ifndef _OBJECT_DATA_H_
#define _OBJECT_DATA_H_

/* _________________________________
   |                                 |
   |           ROS INCLUDES          |
   |_________________________________| */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "pcl17_ros/impl/transforms.hpp"
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/conditional_removal.h>
/* _________________________________
   |                                 |
   |         MY INCLUDES             |
   |_________________________________| */
#include <race_3d_object_tracking/tracker_info.h>

/* _________________________________
   |                                 |
   |           DEFINES               |
   |_________________________________| */

/* _________________________________
   |                                 |
   |        TYPE DEFINITIONS         |
   |_________________________________| */

typedef pcl17::PointXYZRGB Point_T;
typedef pcl17::PointCloud<Point_T>::Ptr PointCloudPtr_T;
typedef pcl17::PointCloud<Point_T> PointCloud_T;


class c_object_data
{
	public:

		c_object_data(size_t obj_num, std::string top_namespace, ros::NodeHandle* n, std_msgs::ColorRGBA c, PointCloudPtr_T pc_in)
		{
			//Define the names structure
			names.obj = "obj" + boost::lexical_cast<std::string>(obj_num);
			names.path_to_pcd = "/tmp/" + names.obj + ".pcd";
			names.tracker_info = top_namespace + names.obj + "/info";
			names.tf = top_namespace + names.obj;

			//define some other variables
			color = c;
			is_pointed_to = false;	
			//centroid = centroid_tmp;
			sphere_radius = 0.1;
			flg.received_first_info_msg = false;


			//Init the subscribers
			ros.sub_info = n->subscribe (names.tracker_info, 1, &c_object_data::callback_tracker_info, this);

			//Init transform listener


			//Write a point cloud pcd to initialize tracking
			pcl17::PCDWriter writer; 
			writer.write<Point_T> (names.path_to_pcd, *pc_in, false);

			//Create the command string to launch a new tracker node
			names.command = "roslaunch race_3d_object_tracking spawn_tracker.launch";
			names.command.append(" object_id:=" + names.obj); //change the node name
			names.command.append(" pc_in:=/openni/depth_registered/points"); 
			names.command.append(" color_r:=" + boost::lexical_cast<std::string>(color.r)); 
			names.command.append(" color_g:=" + boost::lexical_cast<std::string>(color.g)); 
			names.command.append(" color_b:=" + boost::lexical_cast<std::string>(color.b)); 
			names.command.append(" color_a:=0.8 object_pcd_file:=" + names.path_to_pcd); 
			names.command.append(" parent_frame_id:=" + pc_in->header.frame_id); 
			names.command.append(" delta:=0.8 maxparticlenum:=300 epsilon:=0.2 bin:=0.1"); 
			names.command.append("&"); 

			//Launch tracker node
			ROS_INFO("Will execute command \n%s\n", names.command.c_str());

			if (system(names.command.c_str()))
			{
				ROS_INFO("Launching a new tracker ...");
			}


		}


		void callback_tracker_info(const race_3d_object_tracking::tracker_info& msg_in)
		{
			//tracker_info = msg_in; //I am not sure I can do this for any ROS msg
			data.tracker.parent_frame_id = msg_in.parent_frame_id;
			data.tracker.object_frame_id = msg_in.object_frame_id;
			data.tracker.bb_dx = msg_in.bb_dx;
			data.tracker.bb_dy = msg_in.bb_dy;
			data.tracker.bb_dz = msg_in.bb_dz;
			data.tracker.time_since_start = msg_in.time_since_start;
			data.tracker.object_lost = msg_in.object_lost;

			flg.received_first_info_msg = true; //this will allow the class to be aware it can use tracker_info data for additional processing
		}

		bool remove_points_inside_object_bb(PointCloudPtr_T pc_in, PointCloudPtr_T pc_out)
		{
			if (flg.received_first_info_msg==true)
			{
				ROS_INFO("its here");
				//get the transform from the parent_frame to the object's reference frame
				tf::StampedTransform trf;

				ros::Time t = ros::Time::now();
				if(ros.tf_listener.waitForTransform(data.tracker.parent_frame_id, data.tracker.object_frame_id, pc_in->header.stamp, ros::Duration(1.0)))
				{   
					try 
					{   
						ros.tf_listener.lookupTransform(data.tracker.parent_frame_id, data.tracker.object_frame_id, pc_in->header.stamp, trf);
					}   
					catch (tf::TransformException ex) 
					{   
						ROS_ERROR("Could not lookup transform:\n%s",ex.what());
						return false;
					}   
				}   
				else
				{   
					ROS_ERROR("Could find valid transform after waiting 3 secs\n");
					return false;
				}   


				ros::Duration d = ros::Time::now() -t;
				//ROS_INFO("Waited for %f for transform %s to %s", d.toSec(), data.tracker.parent_frame_id.c_str(), data.tracker.object_frame_id.c_str());

				PointCloudPtr_T cloud_in_local = (PointCloudPtr_T)new PointCloud_T;
				PointCloudPtr_T cloud_out_local = (PointCloudPtr_T)new PointCloud_T;

				//Get the cloud_in in local coordinates
				pcl17_ros::transformPointCloud<Point_T>(*pc_in, *cloud_in_local, trf.inverse());
				*cloud_out_local = *cloud_in_local;

				boost::shared_ptr<pcl17::ConditionOr<Point_T> > range_cond(new pcl17::ConditionOr<Point_T>());
				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("x", pcl17::ComparisonOps::LT, -data.tracker.bb_dx/2)));
				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("x", pcl17::ComparisonOps::GT, data.tracker.bb_dx/2)));

				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("y", pcl17::ComparisonOps::LT, -data.tracker.bb_dy/2)));
				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("y", pcl17::ComparisonOps::GT, data.tracker.bb_dy/2)));

				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("z", pcl17::ComparisonOps::LT, -data.tracker.bb_dz/2)));
				range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<Point_T> > 
						(new pcl17::FieldComparison<Point_T> ("z", pcl17::ComparisonOps::GT, data.tracker.bb_dz/2)));

				pcl17::ConditionalRemoval<Point_T> condrem (range_cond);
				condrem.setInputCloud (cloud_out_local);
				condrem.setKeepOrganized(false);
				condrem.filter(*cloud_out_local);

				//transform cloud_out_local back to global coordinates
				pcl17_ros::transformPointCloud<Point_T>(*cloud_out_local, *pc_out, trf);

				cloud_in_local.reset();
				cloud_out_local.reset();
			}
			else
			{
				return false;
			}

			return true;
		}



		struct{
			std::string obj;
			std::string path_to_pcd;
			std::string tracker_info;
			std::string tf;
			std::string command;
		}names;

		struct{

			race_3d_object_tracking::tracker_info tracker;
		}data;




		std_msgs::ColorRGBA color;
		double sphere_radius;
		Eigen::Vector4f centroid;
		bool is_pointed_to;



	private:
		struct{
			ros::Subscriber sub_info;
			tf::TransformListener tf_listener;
		}ros;


		struct{
			bool received_first_info_msg;
		}flg;

};


#endif
