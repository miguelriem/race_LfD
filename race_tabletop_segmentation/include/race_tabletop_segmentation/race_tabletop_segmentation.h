#ifndef _RACE_TABLETOP_SEGMENTATION_H_
#define _RACE_TABLETOP_SEGMENTATION_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tabletop_object_detector/TabletopSegmentation.h>

#include <pc_processing/pc_processing.h>
#include <colormap_utils/colormap_utils.h>

/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */

namespace race_tabletop_segmentor
{

	template <class PointT>
		class RaceTabletopSegmentor: public nodelet::Nodelet 
	{
		public:


			/* _________________________________
			   |                                 |
			   |           VARIABLES             |
			   |_________________________________| */

			struct{
				boost::shared_ptr<pcl17::PointCloud<PointT> > in;
				boost::shared_ptr<pcl17::PointCloud<PointT> > filtered;
				boost::shared_ptr<pcl17::PointCloud<PointT> > downsampled;
				boost::shared_ptr<pcl17::PointCloud<PointT> > downsampled_1;
				boost::shared_ptr<pcl17::PointCloud<PointT> > table_plane;
				boost::shared_ptr<pcl17::PointCloud<PointT> > table;
				boost::shared_ptr<pcl17::PointCloud<PointT> > table2d;
				boost::shared_ptr<pcl17::PointCloud<PointT> > convex_hull;
				boost::shared_ptr<pcl17::PointCloud<PointT> > objects_on_table;
			}pc;
			pcl17::ModelCoefficients::Ptr coefficients;
			std::vector<boost::shared_ptr<pcl17::PointCloud<PointT> > > objects; //a std vector of point cloud smart pointers to store all objects on the table

			//! The node handle
			//ros::NodeHandle nh_;
			ros::NodeHandle* p_nh_;
			ros::NodeHandle nh_;
			//ros::NodeHandle priv_nh_;
			//! Publisher for markers
			ros::Publisher marker_pub_;
			//! Service server for object detection
			ros::ServiceServer segmentation_srv_;
			bool flg_is_nodelet;
			boost::shared_ptr<class_colormap_utils> cm;

			//tf::Transform transf;

			visualization_msgs::MarkerArray marker_array; //variable with the marker array.

			ros::Publisher marker_publisher;

			ros::Publisher pub_pc1;
			ros::Publisher pub_pc2;
			ros::Publisher pub_pc3;
			ros::Publisher pub_pc4;
			ros::Publisher pub_pc5;
			ros::Publisher pub_pc6;
			ros::Publisher pub_pc7;


			/* _________________________________
			   |                                 |
			   |           PARAMETERS			|
			   |_________________________________| */

				double time_to_wait_for_pointcloud_;
				double filter_z_more_than_;
				double plane_voxel_x_, plane_voxel_y_, plane_voxel_z_;
				double object_voxel_x_, object_voxel_y_, object_voxel_z_;
				double inlier_threshold_;
				int ransac_iterations_;
				double table_clustering_step_;
				int table_clustering_min_size_;
				int table_clustering_max_size_;
				double table_z_filter_min_;
				double table_z_filter_max_;
				double object_clustering_step_;
				int object_clustering_min_size_;
				int object_clustering_max_size_;

			/* _________________________________
			   |                                 |
			   |           Methods              |
			   |_________________________________| */


			/**
			 * @brief Constructor When the class is instantiated with an empty
			 * constructor, it is assumed that a nodelet is to be used. Must
			 * call onInit() for setting up the class (as the nodelet manager
			 * does)
			 */
			RaceTabletopSegmentor(){flg_is_nodelet=true;};

			/**
			 * @brief Constructor When the constructor is called with a node
			 * argument, it is assumed that the class will be a node (or inside
			 * some node) (as opposed to a nodelet). In this case the onInit()
			 * is called within the constructor
			 *
			 * @param n a pointer to the node handle of the node
			 */
			RaceTabletopSegmentor(ros::NodeHandle* n)
			{
				flg_is_nodelet=false; 
				p_nh_ = n; 
				onInit();
			};
			~RaceTabletopSegmentor(){};

			virtual void onInit() //Declare here the initialization so that we can use this class as a nodelet
			{

				//Allocate all pcs
				pc.in = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.filtered = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.downsampled = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.downsampled_1 = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.table_plane = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.table = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.table2d = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.convex_hull = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				pc.objects_on_table = (boost::shared_ptr<pcl17::PointCloud<PointT> >) new pcl17::PointCloud<PointT>;
				coefficients = (pcl17::ModelCoefficients::Ptr) new pcl17::ModelCoefficients;

				//Create a new colormap
				cm = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("hot"),5, 1, false);

				//create a node handle in internal nh_ variable, and point p_nh_
				//to it. Only done if we are using a nodelet.
				if (flg_is_nodelet==true)
				{
					nh_ = getNodeHandle(); 
					p_nh_ = &nh_;
				}

				//setup segmentation service
				segmentation_srv_ = p_nh_->advertiseService(p_nh_->resolveName("segmentation_srv"), &RaceTabletopSegmentor::serviceCallback, this);
				//setup rviz marker publisher
				marker_publisher = p_nh_->advertise<visualization_msgs::MarkerArray>("segmentation_markers", 1); 

				pub_pc1 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/distance", 1);
				pub_pc2 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/plane", 1);
				pub_pc3 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/table", 1);
				pub_pc4 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/table2D", 1);
				pub_pc5 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/chull", 1);	
				pub_pc6 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/objects_on_table", 1); 
				pub_pc7 = p_nh_->advertise<sensor_msgs::PointCloud2>("pc_out/untracked_objects_on_table", 1);

				//initialize parameters
				p_nh_->param<double>("inlier_threshold", inlier_threshold_, 0.02);
				p_nh_->param<double>("time_to_wait_for_pointcloud", time_to_wait_for_pointcloud_, 1.0);
				p_nh_->param<double>("filter_z_more_than", filter_z_more_than_, 2.1);
				p_nh_->param<double>("plane_voxel_x", plane_voxel_x_, 0.05);
				p_nh_->param<double>("plane_voxel_y", plane_voxel_y_, 0.05);
				p_nh_->param<double>("plane_voxel_z", plane_voxel_z_, 0.05);
				p_nh_->param<double>("object_voxel_x", object_voxel_x_, 0.01);
				p_nh_->param<double>("object_voxel_y", object_voxel_y_, 0.01);
				p_nh_->param<double>("object_voxel_z", object_voxel_z_, 0.01);
				p_nh_->param<int>("ransac_iterations_", ransac_iterations_, 200);
				p_nh_->param<double>("table_clustering_step", table_clustering_step_, 0.08);
				p_nh_->param<int>("table_clustering_min_size", table_clustering_min_size_, 10);
				p_nh_->param<int>("table_clustering_max_size", table_clustering_max_size_, 640*480 /*max num points in a kinect pc*/);
				p_nh_->param<double>("table_z_filter_min", table_z_filter_min_, 0.02);
				p_nh_->param<double>("table_z_filter_max", table_z_filter_max_, 0.6);
				p_nh_->param<double>("object_clustering_step", object_clustering_step_, 0.08);
				p_nh_->param<int>("object_clustering_min_size", object_clustering_min_size_, 10);
				p_nh_->param<int>("object_clustering_max_size", object_clustering_max_size_, 640*480 /*max num points in a kinect pc*/);

			}

			/**
			 * @brief This service reads a point cloud on topic cloud_in and
			 * detects the table and tabletop objetcs
			 *
			 * @param request
			 * @param response
			 *
			 * @return 
			 */
			bool serviceCallback(tabletop_object_detector::TabletopSegmentation::Request &request, tabletop_object_detector::TabletopSegmentation::Response &response)
			{
				ros::Time start_time = ros::Time::now();
				std::string topic = p_nh_->resolveName("cloud_in");
				ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());
				sensor_msgs::PointCloud2::ConstPtr pcmsg = 
					ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, *p_nh_, ros::Duration(time_to_wait_for_pointcloud_));

				if (!pcmsg)
				{
					ROS_ERROR("Tabletop object detector: no point_cloud2 has been received");
					response.result = response.NO_CLOUD_RECEIVED;
					return true;
				}

				//STEP1: convert from ros msg to pcl (already removes RGB component because IN is pcl17::PointXYZ)
				pcl17::fromROSMsg(*pcmsg, *pc.in);

				ROS_INFO("in has %ld points",pc.in->points.size());

				//STEP2: Remove points based on distance filtering
				if (!filter_along_dimension(pc.in,filter_z_more_than_,"z",pc.filtered)) 
					ROS_ERROR("Could not remove points from point cloud");

				ROS_INFO("in_z_filtered has %ld points",pc.filtered->points.size());

				//STEP3: Downsample input point cloud
				if (!downsample_pc(pc.filtered, plane_voxel_x_, plane_voxel_y_, plane_voxel_z_, pc.downsampled)) 
					ROS_ERROR("Could not downsample point cloud");

				ROS_INFO("in_downsampled has %ld points",pc.downsampled->points.size());

				//STEP4: Downsample input point cloud for segmenting objects later on
				if (!downsample_pc(pc.filtered, object_voxel_x_, object_voxel_y_, object_voxel_z_, pc.downsampled_1)) 
					ROS_ERROR("Could not downsample point cloud");

				ROS_INFO("in_downsampled_1 has %ld points",pc.downsampled_1->points.size());

				//extract points that belong to table plane
				if (!detect_largest_plane(pc.downsampled, inlier_threshold_, pc.table_plane, coefficients,ransac_iterations_)) 
					ROS_ERROR("Could not detect largest plane");

				ROS_INFO("table_plane has %ld points",pc.table_plane->points.size());


				//get largest cluster after clustering
				if (!get_max_group_from_clustering(pc.table_plane, table_clustering_step_, table_clustering_min_size_, table_clustering_max_size_, pc.table))
					ROS_ERROR("Could not get max cluster");

				ROS_INFO("table has %ld points",pc.table->points.size());

				//Compute a transform from in->header.frame_id to the local
				//table coordinate system
				//transf = getPlaneTransform(*coefficients, 1);

				//sensor_msgs::PointCloud table_points;
				//getPlanePoints(pc.table, transf, table_points);


				//Project the model inliers
				if (!project_pc_to_plane(pc.table, coefficients, pc.table2d))
					ROS_ERROR("Projection failed");

				ROS_INFO("table2d has %ld points",pc.table2d->points.size());

				//compute convex hull
				if (!compute_convex_hull(pc.table2d, coefficients, pc.convex_hull))
					ROS_ERROR("Could not compute convex hull");

				ROS_INFO("convex_hull has %ld points",pc.convex_hull->points.size());
				//extract point cloud with all objects on table
				if (!extract_polygonal_prism(pc.downsampled_1, pc.convex_hull, table_z_filter_min_, table_z_filter_max_,  pc.objects_on_table)) 
					ROS_ERROR("Could extract objects on table");

				ROS_INFO("objects_on_table has %ld points (from original pc)",pc.objects_on_table->points.size());

				//extract individual objects on table
				if (!get_all_groups_from_clustering(pc.objects_on_table, object_clustering_step_, object_clustering_min_size_,  object_clustering_max_size_, &objects)) 
					ROS_ERROR("Could not extract objects on table");

				ROS_INFO("There are %ld objects lying on the table",objects.size());


				//Publish stuff to rviz
				marker_array.markers.erase(marker_array.markers.begin(), marker_array.markers.end()); //erase alla previous markers

				//Create the marker for the convex hull line
				visualization_msgs::Marker marker;
				create_line_strip_vizmarker_from_pc(pc.convex_hull, &marker);
				marker.ns = "Table CH";
				marker_array.markers.push_back(marker);

				for (size_t i = 0; i < objects.size(); i++)
				{
					visualization_msgs::Marker marker;
					marker.header.frame_id = "/openni_rgb_optical_frame";
					marker.header.stamp = ros::Time();
					std::stringstream sstm;

					//prepare points marker
					sstm << "Obj" << " points";
					marker.ns = sstm.str();
					marker.id = i;
					marker.type = visualization_msgs::Marker::POINTS;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
					marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
					marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; marker.color.a = 1;

					marker.color = cm->color(i);
					marker.lifetime = ros::Duration(1); //one sec lifetime

					marker.points.erase(marker.points.begin(), marker.points.end());
					geometry_msgs::Point p;
					for (size_t j=0; j<objects.at(i)->points.size(); j++)
					{
						p.x = objects.at(i)->points.at(j).x;
						p.y = objects.at(i)->points.at(j).y;
						p.z = objects.at(i)->points.at(j).z;
						marker.points.push_back(p);
					}

					marker_array.markers.push_back(marker);
				}


				//Publish the marker array with all the markers
				marker_publisher.publish( marker_array );

				//publish the point clouds
				sensor_msgs::PointCloud2 msg; 
				pcl17::toROSMsg(*pc.filtered,msg); 		pub_pc1.publish(msg);
				pcl17::toROSMsg(*pc.table_plane,msg);   pub_pc2.publish(msg);
				pcl17::toROSMsg(*pc.table,msg);         pub_pc3.publish(msg);
				pcl17::toROSMsg(*pc.table2d,msg);       pub_pc4.publish(msg);
				pcl17::toROSMsg(*pc.convex_hull,msg);   pub_pc5.publish(msg);
				pcl17::toROSMsg(*pc.objects_on_table,msg);	pub_pc6.publish(msg);

				ROS_INFO("Finished tabletop segmentation in %f secs",(ros::Time::now()- start_time).toSec());
				return 1;
			}


		private:

			bool getPlanePoints (boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > table, 
					const tf::Transform& table_plane_trans,
					sensor_msgs::PointCloud &table_points)
			{
				// Prepare the output
				table_points.header = table->header;
				table_points.points.resize (table->points.size ());
				for (size_t i = 0; i < table->points.size (); ++i)
				{
					table_points.points[i].x = table->points[i].x;
					table_points.points[i].y = table->points[i].y;
					table_points.points[i].z = table->points[i].z;
				}

				// Transform the data
				tf::TransformListener listener;
				tf::StampedTransform table_pose_frame(table_plane_trans, table->header.stamp, 
						table->header.frame_id, "table_frame");
				listener.setTransform(table_pose_frame);
				std::string error_msg;
				if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
				{
					ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s", 
							table_points.header.frame_id.c_str(), error_msg.c_str());
					return false;
				}
				int current_try=0, max_tries = 3;
				while (1)
				{
					bool transform_success = true;
					try
					{
						listener.transformPointCloud("table_frame", table_points, table_points);
					}
					catch (tf::TransformException ex)
					{
						transform_success = false;
						if ( ++current_try >= max_tries )
						{
							ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
									table_points.header.frame_id.c_str(), ex.what());
							return false;
						}
						//sleep a bit to give the listener a chance to get a new transform
						ros::Duration(0.1).sleep();
					}
					if (transform_success) break;
				}
				table_points.header.stamp = table->header.stamp;
				table_points.header.frame_id = "table_frame";
				return true;
			}


			/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
			tf::Transform getPlaneTransform (pcl17::ModelCoefficients coeffs, double up_direction)
			{
				ROS_ASSERT(coeffs.values.size() > 3);
				double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
				//asume plane coefficients are normalized
				tf::Vector3 position(-a*d, -b*d, -c*d);
				tf::Vector3 z(a, b, c);

				//if we are flattening the plane, make z just be (0,0,up_direction)

				//make sure z points "up"
				ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
				if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
				{
					z = -1.0 * z;
					ROS_INFO("flipped z");
				}

				//try to align the x axis with the x axis of the original frame
				//or the y axis if z and x are too close too each other
				tf::Vector3 x(1, 0, 0);
				if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
				tf::Vector3 y = z.cross(x).normalized();
				x = y.cross(z).normalized();

				tf::Matrix3x3 rotation;
				rotation[0] = x; 	// x
				rotation[1] = y; 	// y
				rotation[2] = z; 	// z
				rotation = rotation.transpose();
				tf::Quaternion orientation;
				rotation.getRotation(orientation);
				ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
				ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
				ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
				return tf::Transform(orientation, position);
			}
	};

}//end race_tabletop_segmentor namespace

#endif
