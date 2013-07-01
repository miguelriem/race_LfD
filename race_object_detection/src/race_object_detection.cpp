#ifndef _RACE_OBJECT_DETECTION_CPP_
#define _RACE_OBJECT_DETECTION_CPP_

#include <race_object_detection/race_object_detection.h>
using namespace race_object_detection;
//Can only be one template class if this is a lib for use as a nodelet
template class RaceObjectDetection<pcl17::PointXYZRGB>;

//std::string segmentation_srv_;
//size_t iteration=0;

/**
 * \brief Callback from the PointCloud subscribed topic
 * \param[in]  const sensor_msgs::PointCloud2
 */
//void callback_pc_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
//{
	//marker_array.markers.erase(marker_array.markers.begin(), marker_array.markers.end());
	//bool flg_continue = true; //a flag to signal if any operation went wrong

//#if 0 [>DEBUG <]
	//ROS_INFO("Received Point Cloud. Processing callback."); 
//#endif

	//if (iteration>0)
		//tabletop_segmentation.request.table = tabletop_segmentation.response.table;

	////Call tabletop segmentation service
	//if (p_client->call(tabletop_segmentation))
	//{
		//ROS_INFO("%s service response:\nTable has frame_id=%s\nposition x=%f y=%f z=%f\norientation x=%f y=%f z=%f w=%f", 
				//segmentation_srv_.c_str(),
				//tabletop_segmentation.response.table.pose.header.frame_id.c_str(), 
				//tabletop_segmentation.response.table.pose.pose.position.x, 
				//tabletop_segmentation.response.table.pose.pose.position.y,
				//tabletop_segmentation.response.table.pose.pose.position.z,
				//tabletop_segmentation.response.table.pose.pose.orientation.x,
				//tabletop_segmentation.response.table.pose.pose.orientation.y,
				//tabletop_segmentation.response.table.pose.pose.orientation.z,
				//tabletop_segmentation.response.table.pose.pose.orientation.w
				//);
	//}
	//else
	//{
		//ROS_ERROR("Failed to call service %s", segmentation_srv_.c_str());
		//return;
	//}

	////erase all objects from the previous iteration
	//OBJECTS.erase(OBJECTS.begin(), OBJECTS.end());

	//[> _________________________________
	   //|                                 |
	   //| REMOVE TRACKED OBJECTS FROM PC  |
	   //|_________________________________| */

	////copy clusters from tabletop object segmentation to OBJECTS, removing
	////already tracked objects
	//for (size_t i=0; i<tabletop_segmentation.response.clusters.size(); i++) 
	//{
		//bool flg_use_this_cluster=true;
		//PointCloudPtr_T pc = (PointCloudPtr_T) new PointCloud_T;
		//sensor_msgs::PointCloud2 msg_pc2;
		//sensor_msgs::convertPointCloudToPointCloud2(tabletop_segmentation.response.clusters.at(i), msg_pc2);
		//pcl17::fromROSMsg( msg_pc2, *pc);

		////ROS_INFO("Before cluster %ld has %ld points",i,pc->points.size());
		//for (size_t j=0; j< obj_data.size(); j++) //cycle all objects that had already been inserted
		//{
			////ROS_INFO("should remove obj%ld from pc",i);
			//if (obj_data.at(j)->data.tracker.object_lost == false)
				//flg_continue = obj_data.at(j)->remove_points_inside_object_bb(pc, pc);

			//if (flg_continue==false) 
			//{
				//ROS_ERROR("Could not remove pc points for tracked obj name %s. Cannot generate new obstacles.",obj_data.at(j)->names.obj.c_str());
				//flg_use_this_cluster=false;
				//break; //if could not remove points do not generate new obstacles
			//}
		//}
		////ROS_INFO("After cluster %ld has %ld points",i,pc->points.size());

		//if (flg_use_this_cluster==true && pc->points.size()>100[>Minimum cluster size<])
		//{
			////	
			//PointCloudPtr_T tmp = (PointCloudPtr_T) new PointCloud_T;
			//OBJECTS.push_back(tmp);
			//pcl17::copyPointCloud<Point_T,Point_T>(*pc, *OBJECTS.at(OBJECTS.size()-1));
		//}
	//}

	//ROS_INFO("New objects to track:\nTabletop detected %ld obstacles\n%ld obstacles are already being tracked\nResult: %ld new candidate objects to track",tabletop_segmentation.response.clusters.size(), tabletop_segmentation.response.clusters.size()-OBJECTS.size(), OBJECTS.size());


	////Compute plane coefficients of the table plane
	//pcl17::ModelCoefficients::Ptr coefficients = (pcl17::ModelCoefficients::Ptr) new pcl17::ModelCoefficients;
	//coefficients->values.resize(4); //to accomodate ABCD Hessian form coefficients
	//if(!get_coefficients_of_tf_XY_plane(p_tf_listener, "/race_tabletop_segmentation/table", pcmsg_in->header.frame_id, ros::Time(0),coefficients))
	//{
		//ROS_ERROR("Could not get table plane coefficients");
		//return;
	//}


	//// _________________________________
	////|                                 |
	////| GENERATE NEW OBJECTS TO TRACK   |
	////|_________________________________| */

	//// initialize object data structure and fill in information
	//for (size_t i=0; i< OBJECTS.size(); i++)
	//{
		//bool flg_stop = false;

		//Eigen::Vector4f centroid_tmp;
		//pcl17::compute3DCentroid (*OBJECTS.at(i), centroid_tmp);
		//pcl17::PointXYZ pt;
		//pt.x = centroid_tmp(0); pt.y = centroid_tmp(1); pt.z = centroid_tmp(2); 

		////CRITERIA: Distance to left hand
		//double distance_to_left_hand;
		//if (!get_distance_point_to_tf_origin(p_tf_listener, OBJECTS.at(i)->header.frame_id, "/left_hand_1", pt, ros::Time(0), &distance_to_left_hand))
			//continue;

		////CRITERIA: Distance to right hand
		//double distance_to_right_hand;
		//if (!get_distance_point_to_tf_origin(p_tf_listener, OBJECTS.at(i)->header.frame_id, "/right_hand_1", pt, ros::Time(0), &distance_to_right_hand))
			//continue;

		////CRITERIA: Distance to table plane
		//double distance_to_table_plane = distance_from_point_to_plane(pt, coefficients);


		////Testing new object candidate
		//if (OBJECTS.at(i)->points.size()> 100 && 
				//OBJECTS.at(i)->points.size() < 450 && 
				//distance_to_table_plane < 0.32 &&
				//distance_to_left_hand > 0.25 &&
				//distance_to_right_hand > 0.25)
		//{

			//ROS_INFO("New candidate object %ld:\nnumber of points %ld\ndistance to table plane =%f\ndistance to left hand %f\ndistance to right hand %f\nresult Aprooved",
					//i,OBJECTS.at(i)->points.size(),distance_to_table_plane, distance_to_left_hand, distance_to_right_hand);


			//size_t obj_num = obj_data.size();
			//std_msgs::ColorRGBA color = cm_object->color(obj_num);

			////now we create an object for inserting into the std vector
			//obj_data.push_back((boost::shared_ptr<c_object_data>) new c_object_data(obj_num, "/tracking/", n,color, OBJECTS.at(i)));
		//}
		//else
		//{
			//ROS_INFO("New candidate object %ld:\nnumber of points %ld\ndistance to table plane =%f\ndistance to left hand %f\ndistance to right hand %f\nresult NOT Aprooved",
					//i,OBJECTS.at(i)->points.size(),distance_to_table_plane, distance_to_left_hand, distance_to_right_hand);
		//}
	//}


	////[> _________________________________
	////|                                 |
	////|     VIRTUAL HAND EXTENSION      |
	////|_________________________________| */

	//////Build a 3d line segment from hand in the direction of the elbow-hand vector
	//////This is a point cloud with only 2 points. line_segment_local 
	////Point_T pt1, pt2;
	////double pointing_size = 0.2; [>meters<]
	////tf::StampedTransform transform;

	////bool flg_compute_VHE = true;
	//////For the second point we need the direction, for that we query the hand to
	//////elbow transformation
	////try
	////{
	////p_tf_listener->lookupTransform("/left_elbow_1", "/left_hand_1", ros::Time(0), transform);
	////}
	////catch (tf::TransformException ex)
	////{
	////ROS_ERROR("%s",ex.what());
	////flg_compute_VHE = false;
	////}	  

	////if (flg_compute_VHE)
	////{
	////double vector[3];
	////vector[0] = transform.getOrigin().x();
	////vector[1] = transform.getOrigin().y();
	////vector[2] = transform.getOrigin().z();
	////normalize_vector(vector);

	//////The first pt is the origin of the hand tf
	////pt1.x =	transform.getOrigin().x();
	////pt1.y =	transform.getOrigin().y();
	////pt1.z =	transform.getOrigin().z();

	////pt2.x = pt1.x + pointing_size * vector[0];
	////pt2.y = pt1.y + pointing_size * vector[1];
	////pt2.z = pt1.z + pointing_size * vector[2];

	//////Now we insert pt1 and pt2 to the line_segment_local point cloud
	////line_segment_local->points.erase( line_segment_local->points.begin(), line_segment_local->points.end());
	////line_segment_local->points.push_back(pt1);
	////line_segment_local->points.push_back(pt2);
	////line_segment_local->header.frame_id = "/left_elbow_1";

	//////Finally, we compute the point cloud in the /openni_rgb_optical_frame for
	//////later comparison with the sphere of the objects
	////try
	////{
	////p_tf_listener->lookupTransform("/left_elbow_1", "/openni_rgb_optical_frame", ros::Time(0), transform);
	////}
	////catch (tf::TransformException ex)
	////{
	////ROS_ERROR("%s",ex.what());
	////}	  

	////pcl17_ros::transformPointCloud(*line_segment_local, *line_segment_global,  transform.inverse());
	////line_segment_global->header.frame_id = "/openni_rgb_optical_frame";

	////[> _________________________________
	////|                                 |
	////| VHE INTERSECTIONS WITH SPHERES  |
	////|_________________________________| */

	////for (size_t i = 0; i < obj_data.size(); i++)
	////{
	////obj_data.at(i)->is_pointed_to = does_line_segment_intersect_sphere( line_segment_global->points.at(0).x, 
	////line_segment_global->points.at(0).y, 
	////line_segment_global->points.at(0).z,
	////line_segment_global->points.at(1).x, 
	////line_segment_global->points.at(1).y, 
	////line_segment_global->points.at(1).z,
	////obj_data.at(i)->centroid[0], obj_data.at(i)->centroid[1], obj_data.at(i)->centroid[2],
	////obj_data.at(i)->sphere_radius);
	////}

	////}


	////[> _________________________________
	////|                                 |
	////|          VISUALIZATION          |
	////|_________________________________| */

	//////publish the point clouds
	////sensor_msgs::PointCloud2 msg; 
	////pcl17::toROSMsg(*IN_Z_FILTERED,msg); 		p_pub_pc1->publish(msg);
	////pcl17::toROSMsg(*TABLE_PLANE,msg);   		p_pub_pc2->publish(msg);
	////pcl17::toROSMsg(*TABLE,msg);         		p_pub_pc3->publish(msg);
	////pcl17::toROSMsg(*TABLE2D,msg);       		p_pub_pc4->publish(msg);
	////pcl17::toROSMsg(*CONVEX_HULL,msg);   		p_pub_pc5->publish(msg);
	////pcl17::toROSMsg(*OBJECTS_ON_TABLE,msg);		p_pub_pc6->publish(msg);
	////pcl17::toROSMsg(*UNTRACKED_OBJECTS_ON_TABLE,msg);		p_pub_pc7->publish(msg);

	////if (OBJECTS.size()>0)
	////{
	////pcl17::toROSMsg(*OBJECTS.at(0),msg);	p_pub_pc_obj1->publish(msg);
	////}

	////for (size_t i = 0; i < OBJECTS.size(); i++)
	////{
	////visualization_msgs::Marker marker;
	////marker.header.frame_id = "/openni_rgb_optical_frame";
	////marker.header.stamp = ros::Time();
	////std::stringstream sstm;

	//////prepare points marker
	////sstm << "Obj" << " points";
	////marker.ns = sstm.str();
	////marker.id = i;
	////marker.type = visualization_msgs::Marker::POINTS;
	////marker.action = visualization_msgs::Marker::ADD;
	////marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
	////marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
	////marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; marker.color.a = 1;

	////marker.color = cm->color(i);
	////marker.lifetime = ros::Duration(1); //one sec lifetime

	////marker.points.erase(marker.points.begin(), marker.points.end());
	////geometry_msgs::Point p;
	////for (size_t j=0; j<OBJECTS.at(i)->points.size(); j++)
	////{
	////p.x = OBJECTS.at(i)->points.at(j).x;
	////p.y = OBJECTS.at(i)->points.at(j).y;
	////p.z = OBJECTS.at(i)->points.at(j).z;
	////marker.points.push_back(p);
	////}

	////marker_array.markers.push_back(marker);
	////}

	////for (size_t i = 0; i < obj_data.size(); i++)
	////{

	////visualization_msgs::Marker marker;

	////std::stringstream sstm;
	//////prepare sphere marker
	////sstm << "Obj" << " sphere";
	////marker.ns = sstm.str();
	////marker.id = i;
	////marker.header.frame_id = "/openni_rgb_optical_frame";
	////marker.header.stamp = ros::Time();
	////marker.ns = "Obj spheres";
	////marker.id = i;
	////marker.type = visualization_msgs::Marker::SPHERE;
	////marker.action = visualization_msgs::Marker::ADD;
	////marker.pose.position.x = obj_data.at(i)->centroid[0];
	////marker.pose.position.y = obj_data.at(i)->centroid[1];
	////marker.pose.position.z = obj_data.at(i)->centroid[2];
	////marker.pose.orientation.x = 0.0;
	////marker.pose.orientation.y = 0.0;
	////marker.pose.orientation.z = 0.0;
	////marker.pose.orientation.w = 1.0;
	////marker.scale.x = obj_data.at(i)->sphere_radius*2;
	////marker.scale.y = obj_data.at(i)->sphere_radius*2;
	////marker.scale.z = obj_data.at(i)->sphere_radius*2;
	////marker.color.a = 0.2;
	////if (!obj_data.at(i)->is_pointed_to)
	////{
	////marker.color.r = 0.0; marker.color.g = 1.0;	marker.color.b = 0.0;
	////}
	////else
	////{
	////marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
	////}
	////marker_array.markers.push_back(marker);
	////}

	//////Create the marker for the convex hull line
	////visualization_msgs::Marker marker;
	////create_line_strip_vizmarker_from_pc(CONVEX_HULL, &marker);
	////marker.ns = "Table CH";
	////marker_array.markers.push_back(marker);

	//////Create the marker for the hand extension tool
	////create_line_strip_vizmarker_from_pc(line_segment_global, &marker);
	////marker.color.r = 1.0; marker.color.g = 0.2; marker.color.b = 1.0;
	////marker.scale.x = 0.01;
	////marker.ns = "Virtual Hand Extension";
	////marker_array.markers.push_back(marker);

	//////Publish the marker array with all the markers
	////p_marker_publisher->publish( marker_array );

	////#if 0 [>DEBUG THIS IS DEBUG OUTPUT<]
	////ROS_INFO("in has %ld points",IN->points.size());
	////ROS_INFO("in_z_filtered has %ld points",IN_Z_FILTERED->points.size());
	////ROS_INFO("in_downsampled has %ld points",IN_DOWNSAMPLED->points.size());
	////ROS_INFO("in_downsampled_1 has %ld points",IN_DOWNSAMPLED_1->points.size());
	////ROS_INFO("table_plane has %ld points",TABLE_PLANE->points.size());
	////ROS_INFO("table has %ld points",TABLE->points.size());
	////ROS_INFO("table2d has %ld points",TABLE2D->points.size());
	////ROS_INFO("convex_hull has %ld points",CONVEX_HULL->points.size());
	////ROS_INFO("objects_on_table has %ld points (from original pc)",OBJECTS_ON_TABLE->points.size());
	////ROS_INFO("There are %ld objects lying on the table",OBJECTS.size());
	////#endif 

	//////if (OBJECTS.size()>0)
	//////{
	//////pcl17::toROSMsg(*OBJECTS.at(0),msg);
	//////std::ostringstream oss; 
	//////oss << (ros::package::getPath("race_learning_by_demonstration")).c_str() << "/pcd/icp_test" << iteration << ".pcd"; 
	////////pcl17::io::savePCDFile (oss.str(), msg);              
	//////}

	//iteration++;
//}


/**
 * @brief 
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
//int main(int argc, char **argv)
//{
	//ros::init(argc, argv, "learning_by_demonstration"); // Initialize ROS coms
	//n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	//n->param<std::string>("segmentation_srv", segmentation_srv_, "/race_tabletop_segmentation/segmentation_srv");


	////Define subscribers
	//ros::Subscriber sub_pc = n->subscribe("/camera/depth_registered/points", 1, callback_pc_received);

	////Initialize the transform listener
	//tf::TransformListener tf_listener; p_tf_listener = &tf_listener;

	////Initialize the transform broadcaster
	//transform_broadcaster = (boost::shared_ptr<tf::TransformBroadcaster>) new tf::TransformBroadcaster;


	////Define publishers and global make pointers aim to them
	//ros::Publisher pub_pc7 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/untracked_objects_on_table", 1); p_pub_pc7 = &pub_pc7;
	//ros::Publisher pub_pc_obj1 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/obj1", 1); p_pub_pc_obj1 = &pub_pc_obj1;
	//ros::Publisher pub_pc_obj2 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/obj2", 1); p_pub_pc_obj2 = &pub_pc_obj2;
	//ros::Publisher pub_pc_obj3 = n->advertise<sensor_msgs::PointCloud2>("/pc_out/obj3", 1); p_pub_pc_obj3 = &pub_pc_obj3;
	//ros::Publisher marker_pub1 = n->advertise<visualization_msgs::Marker>("/table_polygon", 1); p_marker_pub1 = &marker_pub1;
	//ros::Publisher marker_pub2 = n->advertise<visualization_msgs::Marker>("/hand_virtual_extension", 1); p_marker_pub2 = &marker_pub2;
	//ros::Publisher marker_pub3 = n->advertise<visualization_msgs::Marker>("/obj1_center", 1); p_marker_pub3 = &marker_pub3;
	//ros::Publisher marker_publisher = n->advertise<visualization_msgs::MarkerArray>("/LBD", 1); p_marker_publisher = &marker_publisher;
	////ros::Publisher pub_active_object = n->advertise<race_learning_by_demonstration::active_object_list_msg>("/LbD/active_object", 1); p_pub_active_object = &pub_active_object;
	////ros::Publisher marker_publisher1 = n.advertise<visualization_msgs::MarkerArray>("/PF", 1); p_marker_publisher1 = &marker_publisher1;

	//ros::ServiceClient client = n->serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_srv_); p_client = &client;




	//cm = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("hot"),5, 1, false);
	//cm_object = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("lines"),64, 1, false);

	////initialize point cloud map structure pcm
	//pcm.insert(std::make_pair("in",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("in_downsampled",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("in_downsampled_1",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("in_z_filtered",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("table_plane",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("table",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("table2D",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("convex_hull",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("objects_on_table",(PointCloudPtr_T)new PointCloud_T));
	//pcm.insert(std::make_pair("untracked_objects_on_table",(PointCloudPtr_T)new PointCloud_T));

	////initialize the hand pointing point clouds
	//line_segment_local = (PointCloudPtr_T)new PointCloud_T;
	//line_segment_global = (PointCloudPtr_T)new PointCloud_T;

	//ros::Time t = ros::Time::now();
	////Start program
	//ros::Rate loop_rate(20);
	//while (ros::ok())
	//{
		//ros::spinOnce();                   // Handle ROS events
		//loop_rate.sleep();
	//}
	//return 1;
//}

#endif
