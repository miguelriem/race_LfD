#ifndef _RACE_3D_OBJECT_TRACKING_H_
#define _RACE_3D_OBJECT_TRACKING_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

#include <ros/ros.h>

#include <pcl17/common/impl/common.hpp>
#include <pcl17/tracking/tracking.h>
#include <pcl17/tracking/particle_filter.h>
#include <pcl17/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl17/tracking/particle_filter_omp.h>

#include <pcl17/tracking/coherence.h>
#include <pcl17/tracking/distance_coherence.h>
#include <pcl17/tracking/hsv_color_coherence.h>
#include <pcl17/tracking/normal_coherence.h>

#include <pcl17/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl17/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
//#include <pcl17/io/openni_grabber.h>
#include <pcl17/console/parse.h>
#include <pcl17/common/time.h>
#include <pcl17/common/centroid.h>

//#include <pcl17/visualization/cloud_viewer.h>
//#include <pcl17/visualization/pcl_visualizer.h>

#include <pcl17/io/pcd_io.h>

#include <pcl17/filters/passthrough.h>
#include <pcl17/filters/project_inliers.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/approximate_voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/features/integral_image_normal.h>

#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>

#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_polygonal_prism_data.h>
#include <pcl17/segmentation/extract_clusters.h>

#include <pcl17/surface/convex_hull.h>

#include <pcl17/search/pcl_search.h>
#include <pcl17/common/transforms.h>

#include <boost/format.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>


#include <pcl17/filters/conditional_removal.h>

/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */


template <class PointT>
class c_race_3d_object_tracking
{


	public:

		typedef pcl17::PointCloud<PointT> PointCloudT;
		typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
		typedef pcl17::tracking::ParticleFilterTracker<PointT, pcl17::tracking::ParticleXYZRPY> ParticleFilterTrackerT;
		typedef boost::shared_ptr<ParticleFilterTrackerT> ParticleFilterTrackerPtrT;

		/* _________________________________
		   |                                 |
		   |           CONSTRUCTORS          |
		   |_________________________________| */

		c_race_3d_object_tracking(	boost::shared_ptr<tf::TransformBroadcaster> tf_br, 
									std::string parent_frame_id,
									ros::NodeHandle* n,
									std::string name, 
									double delta,
								   	double maxparticlenum,
									double epsilon,
									double bin,
									std_msgs::ColorRGBA color)
		{

			br = tf_br;
			my_tf = name;
			vis_color=color;
			std::string str = name + "/marker";
			parent_tf = parent_frame_id;


			marker_publisher = n->advertise<visualization_msgs::MarkerArray>(str, 1); 

			//initialize point clouds
			cloud_reference = (PointCloudPtrT) new PointCloudT;
			cloud_input = (PointCloudPtrT) new PointCloudT;
			cloud_aligned = (PointCloudPtrT) new PointCloudT;

			marker_array = (boost::shared_ptr<visualization_msgs::MarkerArray>) new visualization_msgs::MarkerArray;

			//initilize tracker
			//Declare and init a OMPT tracker
			//boost::shared_ptr<pcl17::tracking::ParticleFilterOMPTracker<PointT, pcl17::tracking::ParticleXYZRPY> > tracker
			//(new pcl17::tracking::ParticleFilterOMPTracker<PointT, pcl17::tracking::ParticleXYZRPY> (0));
			boost::shared_ptr<pcl17::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, pcl17::tracking::ParticleXYZRPY> > tracker
				(new pcl17::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, pcl17::tracking::ParticleXYZRPY> (0));

			//set tracker parameters
			tracker->setMaximumParticleNum (maxparticlenum);
			tracker->setDelta (delta);
			tracker->setEpsilon (epsilon);
			pcl17::tracking::ParticleXYZRPY bin_size;
			bin_size.x = bin;
			bin_size.y = bin;
			bin_size.z = bin;
			bin_size.roll = bin;
			bin_size.pitch = bin;
			bin_size.yaw = bin;
			tracker->setBinSize (bin_size);

			//point the t pointer to the OMPT tracker
			t = tracker;

			//Set some parameters for the tracker
			std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
			default_step_covariance[3] *= 40.0;
			default_step_covariance[4] *= 40.0;
			default_step_covariance[5] *= 40.0;

			std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
			std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

			t->setTrans(Eigen::Affine3f::Identity ());
			t->setStepNoiseCovariance (default_step_covariance);
			t->setInitialNoiseCovariance (initial_noise_covariance);
			t->setInitialNoiseMean (default_initial_mean);
			t->setIterationNum(1);

			t->setParticleNum (tracker->getMaximumParticleNum()/2);
			t->setResampleLikelihoodThr(0.30);
			t->setUseNormal (false);

			// setup coherences
			boost::shared_ptr<pcl17::tracking::ApproxNearestPairPointCloudCoherence<PointT> > coherence = 
				boost::shared_ptr<pcl17::tracking::ApproxNearestPairPointCloudCoherence<PointT> >
				(new pcl17::tracking::ApproxNearestPairPointCloudCoherence<PointT>());

			//setup distance coherence
			boost::shared_ptr<pcl17::tracking::DistanceCoherence<PointT> > distance_coherence =
				boost::shared_ptr<pcl17::tracking::DistanceCoherence<PointT> > (new pcl17::tracking::DistanceCoherence<PointT> ());
			coherence->addPointCoherence (distance_coherence);

			//setup color coherence
			boost::shared_ptr<pcl17::tracking::HSVColorCoherence<PointT> > color_coherence
				= boost::shared_ptr<pcl17::tracking::HSVColorCoherence<PointT> > (new pcl17::tracking::HSVColorCoherence<PointT> ());
			color_coherence->setWeight (0.2);
			coherence->addPointCoherence (color_coherence);

			//Setup search method (octree for now)
			//boost::shared_ptr<pcl17::search::KdTree<RefPointType> > search (new pcl17::search::KdTree<RefPointType> (false));
			boost::shared_ptr<pcl17::search::Octree<PointT> > search (new pcl17::search::Octree<PointT> (0.01));
			//boost::shared_ptr<pcl17::search::OrganizedNeighbor<RefPointType> > search (new pcl17::search::OrganizedNeighbor<RefPointType>);
			coherence->setSearchMethod (search);
			coherence->setMaximumDistance (0.01);
			t->setCloudCoherence (coherence);

		};

		/* _________________________________
		   |                                 |
		   |           DESCTRUCTORS          |
		   |_________________________________| */


		~c_race_3d_object_tracking(){};



		/* _________________________________
		   |                                 |
		   |           CLASS METHODS         |
		   |_________________________________| */

		int set_obj_to_track(PointCloudPtrT cloud)
		{
			//copy the input cloud to the reference cloud
			cloud_reference = cloud;

			if (parent_tf != cloud->header.frame_id)
				ROS_WARN("The pc_in frame_id does not correspond to the set parent_frame_id");

			//compute the mass center and a transform
			Eigen::Vector4f c;
			pcl17::compute3DCentroid<PointT> (*cloud_reference, c);

			//define initial object location

			Eigen::Affine3f trans = Eigen::Affine3f::Identity();
			trans.translation() = Eigen::Vector3f (c[0], c[1], c[2]);

			//This is an example to change coordinates
			//trans.rotate(Eigen::AngleAxis<float>(1.0, Eigen::Vector3f(0,0,1)));

			//Transform the point cloud
			pcl17::transformPointCloud<PointT> (*cloud_reference, *cloud_reference, trans.inverse());
			t->setTrans (trans);
			t->setMinIndices (int (cloud_reference->points.size ()) / 2);
			t->setReferenceCloud(cloud_reference);

			
			trf.setOrigin(tf::Vector3(trans(0,3), trans(1,3),trans(2,3)));
			Eigen::Quaternionf q(trans.rotation());
			tf::Quaternion Q(q.x(),q.y(),q.z(),q.w()); 
			trf.setRotation(Q); 
			br->sendTransform(tf::StampedTransform(trf, ros::Time::now(), parent_tf, my_tf));

			// min max for bounding box
			pcl17::getMinMax3D(	*cloud_reference, minimum_pt, maximum_pt);

			double extend_bb_ratio=1.2;
			if (fabs(maximum_pt.x) > fabs(minimum_pt.x))
				bb_dx = 2*fabs(maximum_pt.x)*extend_bb_ratio; 
			else
				bb_dx = 2*fabs(minimum_pt.x)*extend_bb_ratio; 

			if (fabs(maximum_pt.y) > fabs(minimum_pt.y))
				bb_dy = 2*fabs(maximum_pt.y)*extend_bb_ratio; 
			else
				bb_dy = 2*fabs(minimum_pt.y)*extend_bb_ratio; 

			if (fabs(maximum_pt.z) > fabs(minimum_pt.z))
				bb_dz = 2*fabs(maximum_pt.z)*extend_bb_ratio; 
			else
				bb_dz = 2*fabs(minimum_pt.z)*extend_bb_ratio; 

			return 1;
		}

		int track_obj(PointCloudPtrT cloud)
		{
			cloud_input = cloud;
			t->setInputCloud (cloud_input);
			t->compute ();
			//double end = pcl17::getTime ();
			//FPS_CALC_END("tracking");
			//tracking_time_ = end - start;

			t->setReferenceCloud(cloud_reference);

			pcl17::tracking::ParticleXYZRPY result = t->getResult ();
			Eigen::Affine3f transformation = t->toEigenMatrix(result);
			pcl17::transformPointCloud<PointT>(*(t->getReferenceCloud()), *cloud_aligned, transformation);

			trf.setOrigin(tf::Vector3(transformation(0,3), transformation(1,3), transformation(2,3)));
			tf::Quaternion Q; 
			Q.setRPY(result.roll, result.pitch, result.yaw); 
			trf.setRotation(Q); 

			ROS_INFO("Sending new tf");
			br->sendTransform(tf::StampedTransform(trf, cloud->header.stamp, parent_tf, my_tf));

		
			return 1;
		}

		int set_visualization_marker_array(void)
		{
			//clear all markers from marker array
			marker_array->markers.erase(marker_array->markers.begin(), marker_array->markers.end());

			//draw particles
			typename pcl17::tracking::ParticleFilterTracker<PointT, pcl17::tracking::ParticleXYZRPY>::PointCloudStatePtr particles = 
				t->getParticles ();

			if (particles)
			{

				//Draw origins of particles
				visualization_msgs::Marker marker;
				marker.header.frame_id = parent_tf;
				marker.header.stamp = ros::Time();
				std::stringstream sstm;

				//prepare points marker
				marker.ns = "particle_origin";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::POINTS;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.008; marker.scale.y = 0.008; marker.scale.z = 1; 
				
				marker.lifetime = ros::Duration(1);
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
			marker.header.stamp = ros::Time();
			std::stringstream sstm;

			//prepare points marker
			marker.ns = "boundingbox";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration(1);

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


		int remove_object_from_point_cloud(PointCloudPtrT cloud_in, PointCloudPtrT cloud_out)
		{
			PointCloudPtrT cloud_in_local = (PointCloudPtrT)new PointCloudT;
			PointCloudPtrT cloud_out_local = (PointCloudPtrT)new PointCloudT;

			//Get the cloud_in in local coordinates
			pcl17::tracking::ParticleXYZRPY result = t->getResult ();
			Eigen::Affine3f transformation = t->toEigenMatrix(result);
			pcl17::transformPointCloud<PointT>(*cloud_in, *cloud_in_local, transformation.inverse());
			*cloud_out_local = *cloud_in_local;


			boost::shared_ptr<pcl17::ConditionOr<PointT> > range_cond(new pcl17::ConditionOr<PointT>());
			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("x", pcl17::ComparisonOps::LT, -bb_dx/2)));
			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("x", pcl17::ComparisonOps::GT, bb_dx/2)));

			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("y", pcl17::ComparisonOps::LT, -bb_dy/2)));
			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("y", pcl17::ComparisonOps::GT, bb_dy/2)));
			
			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("z", pcl17::ComparisonOps::LT, -bb_dz/2)));
			range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<PointT> > 
					(new pcl17::FieldComparison<PointT> ("z", pcl17::ComparisonOps::GT, bb_dz/2)));

			pcl17::ConditionalRemoval<PointT> condrem (range_cond);
			condrem.setInputCloud (cloud_out_local);
			condrem.setKeepOrganized(false);
			condrem.filter(*cloud_out_local);

					
			//transform cloud_out_local back to global coordinates
			pcl17::transformPointCloud<PointT>(*cloud_out_local, *cloud_out, transformation);
		
			return 1;
		}

		/* _________________________________
		   |                                 |
		   |           ACCESSORS             |
		   |_________________________________| */

		PointCloudPtrT get_cloud_aligned(void)
		{
			return cloud_aligned;
		}


		tf::Transform trf;

		double bb_dx, bb_dy, bb_dz;
		std::string parent_tf;
		std::string my_tf;

		ParticleFilterTrackerPtrT t;
		PointCloudPtrT cloud_reference;
		PointCloudPtrT cloud_input;
		PointCloudPtrT cloud_aligned;
		boost::shared_ptr<tf::TransformBroadcaster> br;

		PointT minimum_pt;
		PointT maximum_pt;

		std_msgs::ColorRGBA vis_color;

		boost::shared_ptr<visualization_msgs::MarkerArray> marker_array;
		ros::Publisher marker_publisher;
};



#endif
