#ifndef _RACE_OBJECT_DETECION_H_
#define _RACE_OBJECT_DETECION_H_

/* _________________________________
   |                                 |
   |           ROS INCLUDES          |
   |_________________________________| */
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include "pcl17_ros/impl/transforms.hpp"
#include <pcl17/io/pcd_io.h>
#include <ros/package.h>
#include <boost/lexical_cast.hpp>
#include <pcl17/filters/conditional_removal.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
/* _________________________________
   |                                 |
   |         MY INCLUDES             |
   |_________________________________| */
//#include <pc_processing/pc_processing.h>
#include <colormap_utils/colormap_utils.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>
#include <race_object_detection/object_data.h>
//#include "object_data.h"

#include <tabletop_object_detector/TabletopSegmentation.h>

/* _________________________________
   |                                 |
   |           DEFINES               |
   |_________________________________| */
#define SUCCESS 1
#define DEBUG 1

#ifndef PFLN
#define PFLN printf("L=%d of file: %s \n", __LINE__,__FILE__);
#endif

////_EXTERN_ ros::Publisher* p_pub_active_object;
//_EXTERN_ Map_T pcm; //create a map struct to contain all point clouds
//_EXTERN_ std::vector<PointCloudPtr_T> OBJECTS; //a std vector of point cloud smart pointers to store all objects on the table
//_EXTERN_ tf::TransformListener* p_tf_listener;
//_EXTERN_ PointCloudPtr_T line_segment_local;
//_EXTERN_ PointCloudPtr_T line_segment_global;
//_EXTERN_ visualization_msgs::MarkerArray marker_array; //global variable with the marker array.

//_EXTERN_ std::vector<boost::shared_ptr<c_object_data> > obj_data;

//_EXTERN_ boost::shared_ptr<class_colormap_utils> cm;
//_EXTERN_ boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
//_EXTERN_ ros::NodeHandle* n;
//_EXTERN_ ros::ServiceClient* p_client;

/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */

using namespace std;
using namespace ros;
using namespace tf;

namespace race_object_detection
{

	template <class PointT>
		class RaceObjectDetection: public nodelet::Nodelet 
	{
		public:
			/* _________________________________
			   |                                 |
			   |          LOCAL VARIABLES	     |
			   |_________________________________| */

			ros::NodeHandle* _p_nh; // The pointer to the node handle
			ros::NodeHandle _nh; // The node handle
			ros::NodeHandle _priv_nh; // The node handle
			ros::NodeHandle* _p_priv_nh; // The node handle
			bool _flg_is_nodelet; //a flag to check if this code is running as a node or a nodelet
			size_t _number_of_generated_objects;

			boost::shared_ptr<Subscriber> _p_pcin_subscriber; 
			//boost::shared_ptr<TransformListener> _p_transform_listener; 
			TransformListener* _p_transform_listener; 
			boost::shared_ptr<ServiceClient> _p_tabletop_segmentation_service_client;
			boost::shared_ptr<class_colormap_utils> cm_object;

			string _name;
			tabletop_object_detector::TabletopSegmentation tabletop_segmentation;
			std::vector<PointCloudPtr_T> OBJECTS; //a std vector of point cloud smart pointers to store all objects on the table

			std::vector<boost::shared_ptr<c_object_data> > obj_data;

			string _table_frame_id;
			/* _________________________________
			   |                                 |
			   |           PARAMETERS			|
			   |_________________________________| */

			string _segmentation_service;
			string _point_cloud_in_topic;

			/* _________________________________
			   |                                 |
			   |           CONSTRUCTORS          |
			   |_________________________________| */

			RaceObjectDetection(){_flg_is_nodelet=true;};

			RaceObjectDetection(ros::NodeHandle* n)
			{
				_flg_is_nodelet=false; 
				_p_nh = n; //if this is a node set both the nodehandle and private node handle to n
				_p_priv_nh = n; 
				onInit();
			};


			~RaceObjectDetection()
			{

			};


			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */

			void onInit(void)
			{
				if (_flg_is_nodelet==true)
				{
					_nh = getNodeHandle(); 
					_p_nh = &_nh;
					_priv_nh = getPrivateNodeHandle(); 
					_p_priv_nh = &_priv_nh;
				}

				//set internal variables				
				_name = _p_priv_nh->getNamespace();
				_segmentation_service = _p_priv_nh->resolveName("segmentation_service");
				_point_cloud_in_topic = _p_priv_nh->resolveName("point_cloud_in_topic");
				_table_frame_id = _p_priv_nh->resolveName("table_frame_id");
				_number_of_generated_objects = 0;

				//get params

				//initialize subscribers
				_p_pcin_subscriber = (boost::shared_ptr<Subscriber>) new Subscriber;
				*_p_pcin_subscriber = _p_nh->subscribe (_point_cloud_in_topic, 1, &RaceObjectDetection::point_cloud_in_callback, this);

				//initialize service clients
				_p_tabletop_segmentation_service_client = (boost::shared_ptr<ServiceClient>) new ServiceClient;
				*_p_tabletop_segmentation_service_client = _p_priv_nh->serviceClient<tabletop_object_detector::TabletopSegmentation>(_segmentation_service);
				//initialize transform listener
				//_p_transform_listener = (boost::shared_ptr<TransformListener>) new TransformListener;
				_p_transform_listener = (TransformListener*) new TransformListener;

				//initialize colormaps (to give each object a different markers color)
				cm_object = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("lines"),64, 1, false);

				ROS_INFO("%s initialized\nListening to transforms on topic /tf\nSubscribing to topic %s\nUsing service %s for tabletop segmentation",_name.c_str(), _point_cloud_in_topic.c_str(), _segmentation_service.c_str());

			};

			void point_cloud_in_callback(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
			{
				//marker_array.markers.erase(marker_array.markers.begin(), marker_array.markers.end());
				bool flg_continue = true; //a flag to signal if any operation went wrong

				ROS_INFO("%s:\nReceived Point Cloud. Processing callback.",_name.c_str()); 

				//if (iteration>0)
					//tabletop_segmentation.request.table = tabletop_segmentation.response.table;

				//Call tabletop segmentation service
				if (_p_tabletop_segmentation_service_client->call(tabletop_segmentation))
				{
					ROS_INFO("%s:\n%s service response:\nTable has frame_id=%s\nposition x=%f y=%f z=%f\norientation x=%f y=%f z=%f w=%f", 
							_name.c_str(),
							_segmentation_service.c_str(),
							tabletop_segmentation.response.table.pose.header.frame_id.c_str(), 
							tabletop_segmentation.response.table.pose.pose.position.x, 
							tabletop_segmentation.response.table.pose.pose.position.y,
							tabletop_segmentation.response.table.pose.pose.position.z,
							tabletop_segmentation.response.table.pose.pose.orientation.x,
							tabletop_segmentation.response.table.pose.pose.orientation.y,
							tabletop_segmentation.response.table.pose.pose.orientation.z,
							tabletop_segmentation.response.table.pose.pose.orientation.w
							);
				}
				else
				{
					ROS_ERROR("%s:\nFailed to call service %s", _name.c_str(), _segmentation_service.c_str());
					return;
				}

				//erase all objects from the previous iteration
				OBJECTS.erase(OBJECTS.begin(), OBJECTS.end());

				/* _________________________________
				   |                                 |
				   | REMOVE TRACKED OBJECTS FROM PC  |
				   |_________________________________| */

				//copy clusters from tabletop object segmentation to OBJECTS, removing
				//already tracked objects
				for (size_t i=0; i<tabletop_segmentation.response.clusters.size(); i++) 
				{
					bool flg_use_this_cluster=true;
					PointCloudPtr_T pc = (PointCloudPtr_T) new PointCloud_T;
					sensor_msgs::PointCloud2 msg_pc2;
					sensor_msgs::convertPointCloudToPointCloud2(tabletop_segmentation.response.clusters.at(i), msg_pc2);
					pcl17::fromROSMsg( msg_pc2, *pc);

					//ROS_INFO("Before cluster %ld has %ld points",i,pc->points.size());
					for (size_t j=0; j< obj_data.size(); j++) //cycle all objects that had already been inserted
					{
						//ROS_INFO("should remove obj%ld from pc",i);
						if (obj_data.at(j)->data.tracker.object_lost == false)
							flg_continue = obj_data.at(j)->remove_points_inside_object_bb(pc, pc);

						if (flg_continue==false) 
						{
							ROS_ERROR("%s: Could not remove pc points for tracked obj name %s. Cannot generate new obstacles.",_name.c_str(), obj_data.at(j)->names.obj.c_str());
							flg_use_this_cluster=false;
							break; //if could not remove points do not generate new obstacles
						}
					}
					//ROS_INFO("After cluster %ld has %ld points",i,pc->points.size());

					if (flg_use_this_cluster==true && pc->points.size()>100/*Minimum cluster size*/)
					{
						//	
						PointCloudPtr_T tmp = (PointCloudPtr_T) new PointCloud_T;
						OBJECTS.push_back(tmp);
						pcl17::copyPointCloud<Point_T,Point_T>(*pc, *OBJECTS.at(OBJECTS.size()-1));
					}
				}

				ROS_INFO("%s:\nNew objects to track:\nTabletop detected %ld obstacles\n%ld obstacles are already being tracked\nResult: %ld new candidate objects to track",_name.c_str(), tabletop_segmentation.response.clusters.size(), tabletop_segmentation.response.clusters.size()-OBJECTS.size(), OBJECTS.size());


				////Compute plane coefficients of the table plane
				pcl17::ModelCoefficients::Ptr coefficients = (pcl17::ModelCoefficients::Ptr) new pcl17::ModelCoefficients;
				coefficients->values.resize(4); //to accomodate ABCD Hessian form coefficients
				if(!get_coefficients_of_tf_XY_plane(_p_transform_listener, _table_frame_id, pcmsg_in->header.frame_id, ros::Time(0),coefficients))
				{
					ROS_ERROR("%s:\nCould not get table plane coefficients",_name.c_str());
					return;
				}


				/* _________________________________
				  |                                 |
				  | GENERATE NEW OBJECTS TO TRACK   |
				  |_________________________________| */

				// initialize object data structure and fill in information
				for (size_t i=0; i< OBJECTS.size(); i++)
				{
					bool flg_stop = false;

					Eigen::Vector4f centroid_tmp;
					pcl17::compute3DCentroid (*OBJECTS.at(i), centroid_tmp);
					pcl17::PointXYZ pt;
					pt.x = centroid_tmp(0); pt.y = centroid_tmp(1); pt.z = centroid_tmp(2); 

					//CRITERIA: Distance to left hand
					double distance_to_left_hand;
					if (!get_distance_point_to_tf_origin(_p_transform_listener, OBJECTS.at(i)->header.frame_id, "/left_hand_1", pt, ros::Time(0), &distance_to_left_hand))
						continue;

					//CRITERIA: Distance to right hand
					double distance_to_right_hand;
					if (!get_distance_point_to_tf_origin(_p_transform_listener, OBJECTS.at(i)->header.frame_id, "/right_hand_1", pt, ros::Time(0), &distance_to_right_hand))
						continue;

					//CRITERIA: Distance to table plane
					double distance_to_table_plane = distance_from_point_to_plane(pt, coefficients);


					//Testing new object candidate
					if (OBJECTS.at(i)->points.size()> 100 && 
							OBJECTS.at(i)->points.size() < 450 && 
							distance_to_table_plane < 0.32 &&
							distance_to_left_hand > 0.25 &&
							distance_to_right_hand > 0.25)
					{

						ROS_INFO("%s:\nNew candidate object %ld:\nnumber of points %ld\ndistance to table plane =%f\ndistance to left hand %f\ndistance to right hand %f\nresult Aprooved",_name.c_str(), i,OBJECTS.at(i)->points.size(),distance_to_table_plane, distance_to_left_hand, distance_to_right_hand);

						//size_t obj_num = obj_data.size();
						std_msgs::ColorRGBA color = cm_object->color(_number_of_generated_objects);

						//now we create an object for inserting into the std vector
						obj_data.push_back((boost::shared_ptr<c_object_data>) new c_object_data(_number_of_generated_objects, "/tracking/", _p_priv_nh ,color, OBJECTS.at(i)));
						_number_of_generated_objects++;
					}
					else
					{
						ROS_INFO("%s:New candidate object %ld:\nnumber of points %ld\ndistance to table plane =%f\ndistance to left hand %f\ndistance to right hand %f\nresult NOT Aprooved",_name.c_str(), i,OBJECTS.at(i)->points.size(),distance_to_table_plane, distance_to_left_hand, distance_to_right_hand);
					}
				}


				//[> _________________________________
				//|                                 |
				//|     VIRTUAL HAND EXTENSION      |
				//|_________________________________| */

				////Build a 3d line segment from hand in the direction of the elbow-hand vector
				////This is a point cloud with only 2 points. line_segment_local 
				//Point_T pt1, pt2;
				//double pointing_size = 0.2; [>meters<]
				//tf::StampedTransform transform;

				//bool flg_compute_VHE = true;
				////For the second point we need the direction, for that we query the hand to
				////elbow transformation
				//try
				//{
				//p_tf_listener->lookupTransform("/left_elbow_1", "/left_hand_1", ros::Time(0), transform);
				//}
				//catch (tf::TransformException ex)
				//{
				//ROS_ERROR("%s",ex.what());
				//flg_compute_VHE = false;
				//}	  

				//if (flg_compute_VHE)
				//{
				//double vector[3];
				//vector[0] = transform.getOrigin().x();
				//vector[1] = transform.getOrigin().y();
				//vector[2] = transform.getOrigin().z();
				//normalize_vector(vector);

				////The first pt is the origin of the hand tf
				//pt1.x =	transform.getOrigin().x();
				//pt1.y =	transform.getOrigin().y();
				//pt1.z =	transform.getOrigin().z();

				//pt2.x = pt1.x + pointing_size * vector[0];
				//pt2.y = pt1.y + pointing_size * vector[1];
				//pt2.z = pt1.z + pointing_size * vector[2];

				////Now we insert pt1 and pt2 to the line_segment_local point cloud
				//line_segment_local->points.erase( line_segment_local->points.begin(), line_segment_local->points.end());
				//line_segment_local->points.push_back(pt1);
				//line_segment_local->points.push_back(pt2);
				//line_segment_local->header.frame_id = "/left_elbow_1";

				////Finally, we compute the point cloud in the /openni_rgb_optical_frame for
				////later comparison with the sphere of the objects
				//try
				//{
				//p_tf_listener->lookupTransform("/left_elbow_1", "/openni_rgb_optical_frame", ros::Time(0), transform);
				//}
				//catch (tf::TransformException ex)
				//{
				//ROS_ERROR("%s",ex.what());
				//}	  

				//pcl17_ros::transformPointCloud(*line_segment_local, *line_segment_global,  transform.inverse());
				//line_segment_global->header.frame_id = "/openni_rgb_optical_frame";

				//[> _________________________________
				//|                                 |
				//| VHE INTERSECTIONS WITH SPHERES  |
				//|_________________________________| */

				//for (size_t i = 0; i < obj_data.size(); i++)
				//{
				//obj_data.at(i)->is_pointed_to = does_line_segment_intersect_sphere( line_segment_global->points.at(0).x, 
				//line_segment_global->points.at(0).y, 
				//line_segment_global->points.at(0).z,
				//line_segment_global->points.at(1).x, 
				//line_segment_global->points.at(1).y, 
				//line_segment_global->points.at(1).z,
				//obj_data.at(i)->centroid[0], obj_data.at(i)->centroid[1], obj_data.at(i)->centroid[2],
				//obj_data.at(i)->sphere_radius);
				//}

				//}


				//[> _________________________________
				//|                                 |
				//|          VISUALIZATION          |
				//|_________________________________| */

				////publish the point clouds
				//sensor_msgs::PointCloud2 msg; 
				//pcl17::toROSMsg(*IN_Z_FILTERED,msg); 		p_pub_pc1->publish(msg);
				//pcl17::toROSMsg(*TABLE_PLANE,msg);   		p_pub_pc2->publish(msg);
				//pcl17::toROSMsg(*TABLE,msg);         		p_pub_pc3->publish(msg);
				//pcl17::toROSMsg(*TABLE2D,msg);       		p_pub_pc4->publish(msg);
				//pcl17::toROSMsg(*CONVEX_HULL,msg);   		p_pub_pc5->publish(msg);
				//pcl17::toROSMsg(*OBJECTS_ON_TABLE,msg);		p_pub_pc6->publish(msg);
				//pcl17::toROSMsg(*UNTRACKED_OBJECTS_ON_TABLE,msg);		p_pub_pc7->publish(msg);

				//if (OBJECTS.size()>0)
				//{
				//pcl17::toROSMsg(*OBJECTS.at(0),msg);	p_pub_pc_obj1->publish(msg);
				//}

				//for (size_t i = 0; i < OBJECTS.size(); i++)
				//{
				//visualization_msgs::Marker marker;
				//marker.header.frame_id = "/openni_rgb_optical_frame";
				//marker.header.stamp = ros::Time();
				//std::stringstream sstm;

				////prepare points marker
				//sstm << "Obj" << " points";
				//marker.ns = sstm.str();
				//marker.id = i;
				//marker.type = visualization_msgs::Marker::POINTS;
				//marker.action = visualization_msgs::Marker::ADD;
				//marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
				//marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
				//marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; marker.color.a = 1;

				//marker.color = cm->color(i);
				//marker.lifetime = ros::Duration(1); //one sec lifetime

				//marker.points.erase(marker.points.begin(), marker.points.end());
				//geometry_msgs::Point p;
				//for (size_t j=0; j<OBJECTS.at(i)->points.size(); j++)
				//{
				//p.x = OBJECTS.at(i)->points.at(j).x;
				//p.y = OBJECTS.at(i)->points.at(j).y;
				//p.z = OBJECTS.at(i)->points.at(j).z;
				//marker.points.push_back(p);
				//}

				//marker_array.markers.push_back(marker);
				//}

				//for (size_t i = 0; i < obj_data.size(); i++)
				//{

				//visualization_msgs::Marker marker;

				//std::stringstream sstm;
				////prepare sphere marker
				//sstm << "Obj" << " sphere";
				//marker.ns = sstm.str();
				//marker.id = i;
				//marker.header.frame_id = "/openni_rgb_optical_frame";
				//marker.header.stamp = ros::Time();
				//marker.ns = "Obj spheres";
				//marker.id = i;
				//marker.type = visualization_msgs::Marker::SPHERE;
				//marker.action = visualization_msgs::Marker::ADD;
				//marker.pose.position.x = obj_data.at(i)->centroid[0];
				//marker.pose.position.y = obj_data.at(i)->centroid[1];
				//marker.pose.position.z = obj_data.at(i)->centroid[2];
				//marker.pose.orientation.x = 0.0;
				//marker.pose.orientation.y = 0.0;
				//marker.pose.orientation.z = 0.0;
				//marker.pose.orientation.w = 1.0;
				//marker.scale.x = obj_data.at(i)->sphere_radius*2;
				//marker.scale.y = obj_data.at(i)->sphere_radius*2;
				//marker.scale.z = obj_data.at(i)->sphere_radius*2;
				//marker.color.a = 0.2;
				//if (!obj_data.at(i)->is_pointed_to)
				//{
				//marker.color.r = 0.0; marker.color.g = 1.0;	marker.color.b = 0.0;
				//}
				//else
				//{
				//marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
				//}
				//marker_array.markers.push_back(marker);
				//}

				////Create the marker for the convex hull line
				//visualization_msgs::Marker marker;
				//create_line_strip_vizmarker_from_pc(CONVEX_HULL, &marker);
				//marker.ns = "Table CH";
				//marker_array.markers.push_back(marker);

				////Create the marker for the hand extension tool
				//create_line_strip_vizmarker_from_pc(line_segment_global, &marker);
				//marker.color.r = 1.0; marker.color.g = 0.2; marker.color.b = 1.0;
				//marker.scale.x = 0.01;
				//marker.ns = "Virtual Hand Extension";
				//marker_array.markers.push_back(marker);

				////Publish the marker array with all the markers
				//p_marker_publisher->publish( marker_array );

				//#if 0 [>DEBUG THIS IS DEBUG OUTPUT<]
				//ROS_INFO("in has %ld points",IN->points.size());
				//ROS_INFO("in_z_filtered has %ld points",IN_Z_FILTERED->points.size());
				//ROS_INFO("in_downsampled has %ld points",IN_DOWNSAMPLED->points.size());
				//ROS_INFO("in_downsampled_1 has %ld points",IN_DOWNSAMPLED_1->points.size());
				//ROS_INFO("table_plane has %ld points",TABLE_PLANE->points.size());
				//ROS_INFO("table has %ld points",TABLE->points.size());
				//ROS_INFO("table2d has %ld points",TABLE2D->points.size());
				//ROS_INFO("convex_hull has %ld points",CONVEX_HULL->points.size());
				//ROS_INFO("objects_on_table has %ld points (from original pc)",OBJECTS_ON_TABLE->points.size());
				//ROS_INFO("There are %ld objects lying on the table",OBJECTS.size());
				//#endif 

				////if (OBJECTS.size()>0)
				////{
				////pcl17::toROSMsg(*OBJECTS.at(0),msg);
				////std::ostringstream oss; 
				////oss << (ros::package::getPath("race_learning_by_demonstration")).c_str() << "/pcd/icp_test" << iteration << ".pcd"; 
				//////pcl17::io::savePCDFile (oss.str(), msg);              
				////}

				//iteration++;


			};




	};//end class RaceObjectDetection

	class RaceObjectDetectionNodelet: public RaceObjectDetection<pcl17::PointXYZRGB>{};

	//PLUGINLIB_DECLARE_CLASS(<nodelet namespace>, <nodelet class>, <nodelet namespace>::<nodelet class>, nodelet::Nodelet);
	PLUGINLIB_DECLARE_CLASS(race_object_detection, RaceObjectDetectionNodelet, race_object_detection::RaceObjectDetectionNodelet, nodelet::Nodelet);


}//end race_object_detection namespace

#endif

