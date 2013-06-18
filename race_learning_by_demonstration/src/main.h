#ifndef _MAIN_H_
#define _MAIN_H_

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
#include <pc_processing/pc_processing.h>
#include <colormap_utils/colormap_utils.h>
#include <race_3d_object_tracking/race_3d_object_tracking.h>
#include "object_data.h"

#include <tabletop_object_detector/TabletopSegmentation.h>

/* _________________________________
   |                                 |
   |           DEFINES               |
   |_________________________________| */
#define SUCCESS 1
#define DEBUG 1
#define PFLN printf("L=%d of file: %s \n", __LINE__,__FILE__);

/* _________________________________
   |                                 |
   |        TYPE DEFINITIONS         |
   |_________________________________| */
typedef std::map< std::string, PointCloudPtr_T> Map_T;
typedef std::map< std::string, PointCloudPtr_T>::iterator MapIterator_T;


/* _________________________________
  |                                 |
  |       Function Prototypes       |
  |_________________________________| */

/* _________________________________
   |                                 |
   |           STRUCTS               |
   |_________________________________| */

#ifdef _MAIN_CPP_
#define _EXTERN_ 
#else
#define _EXTERN_ extern
#endif

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */

_EXTERN_ tabletop_object_detector::TabletopSegmentation tabletop_segmentation;
_EXTERN_ ros::Publisher* p_pub_pc1;
_EXTERN_ ros::Publisher* p_pub_pc2;
_EXTERN_ ros::Publisher* p_pub_pc3;
_EXTERN_ ros::Publisher* p_pub_pc4;
_EXTERN_ ros::Publisher* p_pub_pc5;
_EXTERN_ ros::Publisher* p_pub_pc6;
_EXTERN_ ros::Publisher* p_pub_pc7;
_EXTERN_ ros::Publisher* p_pub_pc_obj1;
_EXTERN_ ros::Publisher* p_pub_pc_obj2;
_EXTERN_ ros::Publisher* p_pub_pc_obj3;
_EXTERN_ ros::Publisher* p_marker_pub1;
_EXTERN_ ros::Publisher* p_marker_pub2;
_EXTERN_ ros::Publisher* p_marker_pub3;
_EXTERN_ ros::Publisher* p_marker_publisher;
_EXTERN_ ros::Publisher* p_marker_publisher1;
//_EXTERN_ ros::Publisher* p_pub_active_object;
_EXTERN_ Map_T pcm; //create a map struct to contain all point clouds
_EXTERN_ std::vector<PointCloudPtr_T> OBJECTS; //a std vector of point cloud smart pointers to store all objects on the table
_EXTERN_ tf::TransformListener* p_tf_listener;
_EXTERN_ PointCloudPtr_T line_segment_local;
_EXTERN_ PointCloudPtr_T line_segment_global;
_EXTERN_ visualization_msgs::MarkerArray marker_array; //global variable with the marker array.

_EXTERN_ std::vector<boost::shared_ptr<c_object_data> > obj_data;

_EXTERN_ boost::shared_ptr<class_colormap_utils> cm;
_EXTERN_ boost::shared_ptr<class_colormap_utils> cm_object;
_EXTERN_ boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
_EXTERN_ ros::NodeHandle* n;
_EXTERN_ ros::ServiceClient* p_client;
#endif

