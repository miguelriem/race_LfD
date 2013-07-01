#ifndef _RACE_3D_OBJECT_TRACKING_CPP_
#define _RACE_3D_OBJECT_TRACKING_CPP_

#include <race_3d_object_tracking/race_3d_object_tracking.h>
using namespace race_3d_object_tracking;
//template class c_race_3d_object_tracking<pcl17::PointXYZ>;
//Can only be one template class if this is a lib for use as a nodelet
template class c_race_3d_object_tracking<pcl17::PointXYZRGB>;

#endif
