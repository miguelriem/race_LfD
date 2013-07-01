#ifndef _RACE_3D_OBJECT_TRACKING_FUNCTIONS_CPP_
#define _RACE_3D_OBJECT_TRACKING_FUNCTIONS_CPP_
#include <race_3d_object_tracking/race_3d_object_tracking.h>

using namespace race_3d_object_tracking;

int c_race_3d_object_tracking::track_obj(PointCloudPtrT cloud)
{
	cloud_input = cloud;
	t->setInputCloud (cloud_input);
	t->compute ();
	//double end = getTime ();
	//FPS_CALC_END("tracking");
	//tracking_time_ = end - start;

	t->setReferenceCloud(cloud_reference);

	tracking::ParticleXYZRPY result = t->getResult ();
	Eigen::Affine3f transformation = t->toEigenMatrix(result);
	transformPointCloud<PointT>(*(t->getReferenceCloud()), *cloud_aligned, transformation);

	trf.setOrigin(Vector3(transformation(0,3), transformation(1,3), transformation(2,3)));
	Quaternion Q; 
	Q.setRPY(result.roll, result.pitch, result.yaw); 
	trf.setRotation(Q); 

	ROS_INFO("Sending new tf");
	br->sendTransform(StampedTransform(trf, cloud->header.stamp, parent_tf, my_tf));


	return 1;
}


template <class PointT>
int c_race_3d_object_tracking::set_visualization_marker_array(void)
{
	//clear all markers from marker array
	marker_array->markers.erase(marker_array->markers.begin(), marker_array->markers.end());

	//draw particles
	typename tracking::ParticleFilterTracker<PointT, tracking::ParticleXYZRPY>::PointCloudStatePtr particles = 
		t->getParticles ();

	if (particles)
	{

		//Draw origins of particles
		visualization_msgs::Marker marker;
		marker.header.frame_id = parent_tf;
		marker.header.stamp = Time();
		stringstream sstm;

		//prepare points marker
		marker.ns = "particle_origin";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; 

		marker.lifetime = Duration(1);
		marker.color = vis_color;
		marker.points.erase(marker.points.begin(), marker.points.end());
		geometry_msgs::Point p;

		//ROS_WARN("There are %ld particles in the filter",particles->points.size ());
		for (size_t i = 0; i < particles->points.size (); i++)
		{
			p.x = particles->points[i].x;
			p.y = particles->points[i].y;
			p.z = particles->points[i].z;
			marker.points.push_back(p);
		}

		marker_array->markers.push_back(marker);

	}
	else
	{
		ROS_WARN("There are no particles in the filter");
	}

	//Draw origins of particles
	visualization_msgs::Marker marker;

	marker.header.frame_id = my_tf;
	marker.header.stamp = Time();
	stringstream sstm;

	//prepare points marker
	marker.ns = "boundingbox";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = Duration(1);

	marker.pose.position.x = 0;	
	marker.pose.position.y = 0;	
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0; 
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0; 
	marker.pose.orientation.w = 1.0;

	marker.scale.x = bb_dx; 
	marker.scale.y = bb_dy; 
	marker.scale.z = bb_dz; 

	marker.color = vis_color;
	vis_color.a = 0.2;

	marker_array->markers.push_back(marker);
	marker_publisher.publish(marker_array);
	return 1;
}

template <class PointT>
int c_race_3d_object_tracking::remove_object_from_point_cloud(PointCloudPtrT cloud_in, PointCloudPtrT cloud_out)
{
	PointCloudPtrT cloud_in_local = (PointCloudPtrT)new PointCloudT;
	PointCloudPtrT cloud_out_local = (PointCloudPtrT)new PointCloudT;

	//Get the cloud_in in local coordinates
	tracking::ParticleXYZRPY result = t->getResult ();
	Eigen::Affine3f transformation = t->toEigenMatrix(result);
	transformPointCloud<PointT>(*cloud_in, *cloud_in_local, transformation.inverse());
	*cloud_out_local = *cloud_in_local;


	shared_ptr<ConditionOr<PointT> > range_cond(new ConditionOr<PointT>());
	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("x", ComparisonOps::LT, -bb_dx/2)));
	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("x", ComparisonOps::GT, bb_dx/2)));

	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("y", ComparisonOps::LT, -bb_dy/2)));
	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("y", ComparisonOps::GT, bb_dy/2)));

	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("z", ComparisonOps::LT, -bb_dz/2)));
	range_cond->addComparison (shared_ptr< const FieldComparison<PointT> > 
			(new FieldComparison<PointT> ("z", ComparisonOps::GT, bb_dz/2)));

	ConditionalRemoval<PointT> condrem (range_cond);
	condrem.setInputCloud (cloud_out_local);
	condrem.setKeepOrganized(false);
	condrem.filter(*cloud_out_local);


	//transform cloud_out_local back to global coordinates
	transformPointCloud<PointT>(*cloud_out_local, *cloud_out, transformation);

	return 1;
}


#endif
