#ifndef _GESTURE_DETECTION_NODE_H_
#define _GESTURE_DETECTION_NODE_H_

/* _________________________________
   |                                 |
   |           ROS INCLUDES          |
   |_________________________________| */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include "pcl17_ros/impl/transforms.hpp"
#include <pcl17/io/pcd_io.h>
#include <ros/package.h>
#include <boost/lexical_cast.hpp>
#include <pcl17/filters/conditional_removal.h>
/* _________________________________
   |                                 |
   |         MY INCLUDES             |
   |_________________________________| */
#include <pc_processing/pc_processing.h>
#include <race_learning_by_demonstration/active_object_list_msg.h>

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


/* _________________________________
  |                                 |
  |       Function Prototypes       |
  |_________________________________| */

/* _________________________________
   |                                 |
   |           STRUCTS               |
   |_________________________________| */


#ifdef _GESTURE_DETECTION_NODE_CPP_
#define _EXTERN_ 
#else
#define _EXTERN_ extern
#endif

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */
_EXTERN_ tf::TransformListener* p_tf_listener;
_EXTERN_ boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > line_segment_local;
_EXTERN_ boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > line_segment_global;
_EXTERN_ visualization_msgs::MarkerArray marker_array; //global variable with the marker array.
_EXTERN_ ros::NodeHandle* n;
#endif

