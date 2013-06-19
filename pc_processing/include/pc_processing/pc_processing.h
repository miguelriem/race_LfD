#ifndef _PC_PROCESSING_H_
#define _PC_PROCESSING_H_
#endif

//ROS includes
#include <ros/ros.h>
#include "pcl17_ros/impl/transforms.hpp"
#include <pcl17/ros/conversions.h>
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl17/filters/conditional_removal.h>
#include <pcl17/sample_consensus/ransac.h>
#include <pcl17/sample_consensus/sac_model_plane.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/kdtree/kdtree.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_clusters.h>
#include <pcl17/filters/project_inliers.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/surface/convex_hull.h>
#include <pcl17/segmentation/extract_polygonal_prism_data.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

// CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include<CGAL/Polygon_2.h>
#include <CGAL/Exact_spherical_kernel_3.h> //for the spherical kernel intersections
#include <CGAL/Spherical_kernel_intersections.h> //for the spherical kernel intersections


	template <typename T>
bool get_distance_point_to_tf_origin(   tf::TransformListener* p_tf_listener, 
		std::string point_frame_id, 
		std::string tf_frame_id,
		T pt, 
		ros::Time t,
		double *distance)
{
	tf::StampedTransform trf;

	try 
	{   
		p_tf_listener->lookupTransform(point_frame_id, tf_frame_id, t, trf);
	}   
	catch (tf::TransformException ex) 
	{   
		ROS_ERROR("%s",ex.what());
		return 0;
	}         

	pcl17::PointXYZ tf_origin;
	tf_origin.x = trf.getOrigin().x(); 
	tf_origin.y = trf.getOrigin().y();
	tf_origin.z = trf.getOrigin().z();
	*distance = sqrt(   (tf_origin.x-pt.x)*(tf_origin.x-pt.x) +
			(tf_origin.y-pt.y)*(tf_origin.y-pt.y) +
			(tf_origin.z-pt.z)*(tf_origin.z-pt.z));

	return 1;
}


	template <typename T>
double distance_from_point_to_plane(T pt, pcl17::ModelCoefficients::Ptr coeff)
{
	double a = coeff->values[0]; double b = coeff->values[1]; double c = coeff->values[2];
	double d = coeff->values[3];
	double x0 = pt.x; double y0 = pt.y; double z0 = pt.z;

	return (fabs(a*x0 + b*y0 + c*z0 + d))/sqrt(a*a + b*b + c*c);
}




/**
 * @brief Returns true if a line segment and a sphere intersect. False
 * otherwise. Line segment is defined by point A (ptA) and point B (ptB). The
 * sphere is defined by the coordinates of its center and the radius.
 *
 * @param ptAx x coordinate of point A.
 * @param ptAy y coordinate of point A.
 * @param ptAz z coordinate of point A.
 * @param ptBx x coordinate of point B.
 * @param ptBy x coordinate of point B.
 * @param ptBz x coordinate of point B.
 * @param cx x coordinate of the center of the sphere.
 * @param cy y coordinate of the center of the sphere.
 * @param cz z coordinate of the center of the sphere.
 * @param radius the radius of the sphere.
 *
 * @return Returns true if a line segment and a sphere intersect. False otherwise.

*/
bool does_line_segment_intersect_sphere(double ptAx, double ptAy, double ptAz,
		double ptBx, double ptBy, double ptBz,
		double cx, double cy, double cz,
		double radius)
{
	//Create points A and B
	CGAL::Exact_spherical_kernel_3::Point_3 cgal_pt1(ptAx, ptAy, ptAz);
	CGAL::Exact_spherical_kernel_3::Point_3 cgal_pt2(ptBx, ptBy, ptBz);

	//Create the line_arc_3
	CGAL::Exact_spherical_kernel_3::Line_arc_3 cgal_seg = CGAL::Exact_spherical_kernel_3::Line_arc_3(
			CGAL::Exact_spherical_kernel_3::Line_3(cgal_pt1, cgal_pt2), cgal_pt1, cgal_pt2);	

	//Create the sphere
	CGAL::Exact_spherical_kernel_3::Sphere_3 cgal_sphere = 
		CGAL::Exact_spherical_kernel_3::Sphere_3(CGAL::Exact_spherical_kernel_3::Point_3(cx,cy,cz),
				radius*radius); //The constructor for the sphere wants the square radius

	std::vector< CGAL::Object > intersecs;
	CGAL::intersection(cgal_seg, cgal_sphere,std::back_inserter(intersecs));

	if(intersecs.size() > 0)	
		return true;
	else
		return false;

}

int get_XYZ_from_XYZRGB_pointcloud(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> >  pc_in, 
		boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >  pc_out)
{
	pc_out->points.resize(pc_in->size());
	for (size_t i = 0; i < pc_in->points.size(); i++) 
	{
		pc_out->points[i].x = pc_in->points[i].x;
		pc_out->points[i].y = pc_in->points[i].y;
		pc_out->points[i].z = pc_in->points[i].z;
	}

	pc_out->header.stamp = pc_in->header.stamp;
	pc_out->header.frame_id = pc_in->header.frame_id;
	pc_out->height = pc_in->height;
	pc_out->width = pc_in->width;
	return 1;
}

/**
 * @brief 
 *
 * @tparam T
 * @param pc_in
 * @param threshold
 * @param dimension
 * @param pc_out
 *
 * @return 
 */
	template <typename T>
int filter_along_dimension(boost::shared_ptr<pcl17::PointCloud<T> >  pc_in,
		double threshold,
		std::string dimension,
		boost::shared_ptr<pcl17::PointCloud<T> >  pc_out)
{

	//ROS_INFO("before cleanup %ld points", pc_in->points.size());

	boost::shared_ptr<pcl17::ConditionAnd<T> > range_cond(new pcl17::ConditionAnd<T>());

	range_cond->addComparison (boost::shared_ptr< const pcl17::FieldComparison<T> > 
			(new pcl17::FieldComparison<T> (dimension.c_str(), pcl17::ComparisonOps::LT, threshold)));

	pcl17::ConditionalRemoval<T> condrem (range_cond);
	condrem.setInputCloud (pc_in);
	condrem.setKeepOrganized(false);
	condrem.filter(*pc_out);

	//ROS_INFO("after cleanup %ld points", pc_out->points.size());

	return 1;
};


	template <typename T>
int detect_largest_plane(boost::shared_ptr<pcl17::PointCloud<T> >  pc_in,
		double threshold,
		boost::shared_ptr<pcl17::PointCloud<T> >  pc_out,
		boost::shared_ptr<pcl17::ModelCoefficients> coefficients,
		int max_iterations
		)
{
	pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
	pcl17::SACSegmentation<T> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType (pcl17::SACMODEL_PLANE);
	seg.setMethodType (pcl17::SAC_RANSAC);
	seg.setMaxIterations(max_iterations);
	seg.setDistanceThreshold (threshold);
	seg.setInputCloud(pc_in->makeShared ());
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		ROS_ERROR("Could not estimate a planar model for the given dataset.");
		return (0);
	}

	pcl17::ExtractIndices<T> extract;
	extract.setInputCloud(pc_in);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*pc_out);

	return 1;
};


	template <typename T>
int create_line_strip_vizmarker_from_pc(boost::shared_ptr<pcl17::PointCloud<T> >  pc_in,
		visualization_msgs::Marker* marker)
{

	marker->header.frame_id = pc_in->header.frame_id;
	marker->header.stamp = ros::Time();
	marker->ns = "";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::LINE_STRIP;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = 0;
	marker->pose.position.y = 0;
	marker->pose.position.z = 0;
	marker->pose.orientation.x = 0.0;
	marker->pose.orientation.y = 0.0;
	marker->pose.orientation.z = 0.0;
	marker->pose.orientation.w = 1.0;
	marker->scale.x = 0.01;
	marker->scale.y = 0.1;
	marker->scale.z = 0.1;
	marker->color.a = 1.0;
	marker->color.r = 0.0;
	marker->color.g = 1.0;
	marker->color.b = 0.0;
	geometry_msgs::Point p;

	marker->points.erase(marker->points.begin(), marker->points.end());
	for (size_t i=0; i<pc_in->points.size(); i++)
	{
		p.x = pc_in->points.at(i).x;
		p.y = pc_in->points.at(i).y;
		p.z = pc_in->points.at(i).z;
		marker->points.push_back(p);
	}

	return 1;
};


/**
 * @brief Normalizes a vector
 *
 * @param v the vector to normalize v[0]=x , v[1]=y, v[2]=z
 */
void normalize_vector(double *v)
{
	double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] = v[0]/n;
	v[1] = v[1]/n;
	v[2] = v[2]/n;
}

/**
 * @brief 
 *
 * @tparam T
 * @param pcin
 * @param coeff
 * @param pcout
 */
	template <typename T>
int project_pc_to_plane(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, boost::shared_ptr<pcl17::ModelCoefficients> coefficients, boost::shared_ptr<pcl17::PointCloud<T> > pc_out)
{
	//Create the projection object
	pcl17::ProjectInliers<T> projection;

	projection.setModelType(pcl17::SACMODEL_NORMAL_PLANE); //set model type
	projection.setInputCloud(pc_in);
	projection.setModelCoefficients(coefficients);
	projection.filter(*pc_out);
	return 1;
}

/**
 * @brief Projects all the points in a point cloud ptin to the plane defined by coeff, and writes the result to ptout
 *
 * @param ptin the input point cloud
 * @param coeff the projection plane
 * @param ptout the output point cloud
 */
	template <typename T>
int project_point_to_plane(T* ptin, pcl17::ModelCoefficients::Ptr coeff, T* ptout)
{
	boost::shared_ptr<pcl17::PointCloud<T> > pcin = boost::shared_ptr<pcl17::PointCloud<T> > (new pcl17::PointCloud<T>);
	pcin->points.push_back(*ptin);

	boost::shared_ptr<pcl17::PointCloud<T> > pcout = boost::shared_ptr<pcl17::PointCloud<T> > (new pcl17::PointCloud<T>);
	pcl17::ProjectInliers<T> projection; //Create the projection object
	projection.setModelType(pcl17::SACMODEL_PLANE); //set model type
	projection.setInputCloud(pcin);
	projection.setModelCoefficients(coeff);
	projection.filter(*pcout);

	*ptout = pcout->points[0];

	pcin.reset();
	pcout.reset();
	return 1;
}


/**
 * @brief Creates an arbitrary reference frame from a plane equation and two inliers. The Z axis is given by the vector normal to the plane, the X axis is defined by the vector to goes from pt1 to pt2. Y is defined by the external product Z*X.
 *
 * @param plane the plane that defines the Z axis
 * @param pt1 The first point
 * @param pt2 The second point
 * @param The output reference frame
 */
	template <typename T>
void create_reference_frame_from_plane_and_two_points(
		pcl17::ModelCoefficients::Ptr plane,
		T *pt1,
		T *pt2,
		tf::Transform* transf)
{
	//STEP1. Project the two points to the plane
	T pt1_projected, pt2_projected;
	project_point_to_plane(pt1, plane, &pt1_projected);
	project_point_to_plane(pt2, plane, &pt2_projected);

	//STEP2. Check if the points are cohincident. If so cannot compute
	if (pt1_projected.x == pt2_projected.x && 
			pt1_projected.y == pt2_projected.y && 
			pt1_projected.z == pt2_projected.z)	
	{
		ROS_ERROR("Cannot create reference frame. pt1 and pt2 after projection are cohincident.");
	}

	//STEP3. find the nsa vectors. a is given by the normal vector to the plane
	double a[3] = {plane->values[0],plane->values[1],plane->values[2]};

	//STEP4. Find the nsa vectors. the n vector will be the vector from pt1_projected to pt2_projected 
	double n[3]={pt2_projected.x - pt1_projected.x, pt2_projected.y - pt1_projected.y, pt2_projected.z - pt1_projected.z};

	//STEP5. Find the nsa vectors. The s vector is given by the external product a*n
	double s[3]={a[1]*n[2] - a[2]*n[1],	a[2]*n[0] - a[0]*n[2], a[0]*n[1] - a[1]*n[0]};

	//STEP6. Normalize all vectors
	normalize_vector(n); normalize_vector(s); normalize_vector(a);

	//STEP7. Compute the transform. The orientation given by nsa and the origin by pt1
	*transf = tf::Transform(tf::Matrix3x3(n[0], s[0] , a[0],
				n[1], s[1] , a[1],  
				n[2], s[2] , a[2]),  
			tf::Vector3(pt1_projected.x,
				pt1_projected.y,
				pt1_projected.z));
}

	template <typename T>
int compute_convex_hull(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, 
		pcl17::ModelCoefficients::Ptr coeff, 
		boost::shared_ptr<pcl17::PointCloud<T> > ch_out)
{

	//use CGAL library convex hull computation. For some reason pcl convex hull
	//function outputs a weird result with the triangles that belong to a mesh.
	//Anyway, this works just fine so I use CGAL

	//To compute the convex hull the following steps are required:
	//1. project the points from the input point cloud pc_in to the plane model (coeff)
	//2. Compute the convex hull on these projected points and store to pcout

	boost::shared_ptr<pcl17::PointCloud<T> > pc_projected = boost::shared_ptr<pcl17::PointCloud<T> > 
		(new pcl17::PointCloud<T>);
	boost::shared_ptr<pcl17::PointCloud<T> > pc_local = boost::shared_ptr<pcl17::PointCloud<T> > 
		(new pcl17::PointCloud<T>);
	boost::shared_ptr<pcl17::PointCloud<T> > pc_ch_local = boost::shared_ptr<pcl17::PointCloud<T> > 
		(new pcl17::PointCloud<T>);

	pc_projected->header.frame_id = pc_in->header.frame_id;
	pc_local->header.frame_id = "/tf_local";
	pc_ch_local->header.frame_id = "/tf_local";

	//STEP 1. the projection
	project_pc_to_plane(pc_in, coeff, pc_projected);

	//STEP2. transforming to local frame
	tf::Transform transf;
	T pt1 = pc_projected->points.at(0);
	T pt2 = pc_projected->points.at(1);
	create_reference_frame_from_plane_and_two_points(coeff,&pt1, &pt2, &transf);

	//the given transform goes from local to global. We want to change from global to local. Hence we use the inverse transform
	pcl17_ros::transformPointCloud(*pc_projected, *pc_local,  transf.inverse());

	//STEP3. Copy the local to a pts vector
	std::vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> pts; //The polygon

	////Setup the polygon with the points from local
	for (int i=0; i< (int) pc_local->size(); i++)
	{
		pts.push_back(CGAL::Exact_predicates_inexact_constructions_kernel::Point_2(pc_local->points[i].x, pc_local->points[i].y));
	}

	std::vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> pts_ch; //The ch polygon

	CGAL::convex_hull_2( pts.begin(), pts.end(),std::back_inserter(pts_ch));
	for (int t = 0; t<(int)pts_ch.size(); t++ )
	{
		T point; 
		point.x = 	pts_ch[t].x();
		point.y = 	pts_ch[t].y();
		point.z = 	0;
		pc_ch_local->points.push_back(point);
	}

	//STEP 3. Transform pc local extended to pcout
	pcl17_ros::transformPointCloud(*pc_ch_local, *ch_out, transf);
	ch_out->header.frame_id = pc_in->header.frame_id;
	pc_projected.reset();
	pc_local.reset();
	pc_ch_local.reset();

	return 1;
}

	template <typename T>
int downsample_pc(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, double dx, double dy, double dz, boost::shared_ptr<pcl17::PointCloud<T> > pc_out)
{
	pcl17::VoxelGrid<T> sor;
	sor.setInputCloud (pc_in);
	sor.setLeafSize (dx, dy, dz);
	sor.filter (*pc_out);
	return 1;
}

/**
 * @brief 
 *
 * @tparam T
 * @param pc_in
 * @param tolerance
 * @param min_size
 * @param max_size
 * @param pc_out
 *
 * @return 
 */
	template <typename T>
int get_max_group_from_clustering(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, 
		double tolerance,
		size_t min_size,
		size_t max_size, 
		boost::shared_ptr<pcl17::PointCloud<T> > pc_out)
{

	boost::shared_ptr<pcl17::search::KdTree<T> > tree (new pcl17::search::KdTree<T>);
	tree->setInputCloud(pc_in);
	std::vector<pcl17::PointIndices> cluster_indices;
	pcl17::EuclideanClusterExtraction<T> ec;
	ec.setClusterTolerance (tolerance); 
	ec.setMinClusterSize (min_size);
	ec.setMaxClusterSize (max_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pc_in);
	ec.extract(cluster_indices);

	size_t max_pts =0;
	int max_index = -1;
	int j=0;
	for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it !=cluster_indices.end (); ++it)
	{
		//std::cout <<"PointCloud	representing the Cluster " << j << " has: " << it->indices.size() << " data points."<< std::endl;
		if (j==0)
		{
			max_index=0;
			max_pts = it->indices.size();
		}
		else if (max_pts< it->indices.size())
		{
			max_index=j;
			max_pts = it->indices.size();
		}
		j++;
	}

	//ROS_INFO("Max cluster index is computed as %d",max_index);

	j=0;
	//clear points from pc_out
	pc_out->points.erase(pc_out->points.begin(), pc_out->points.end());
	for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it !=cluster_indices.end (); ++it)
	{
		if (j==max_index)
		{
			for (std::vector<int>::const_iterator pit =	it->indices.begin(); pit != it->indices.end (); pit++)
			{
				pc_out->points.push_back	(pc_in->points[*pit]); 
			}
			pc_out->width = pc_out->points.size();
			pc_out->height = 1;
			pc_out->header.frame_id = pc_in->header.frame_id;
			pc_out->is_dense = true;
		}
		j++;
	}

	if (pc_out->points.size()<5)
		return 0;
	else
		return 1;
}

	template <typename T>
int get_all_groups_from_clustering(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, 
		double tolerance,
		size_t min_size,
		size_t max_size, 
		std::vector< boost::shared_ptr<pcl17::PointCloud<T> > >* clusters)
{

	boost::shared_ptr<pcl17::search::KdTree<T> > tree (new pcl17::search::KdTree<T>);
	tree->setInputCloud(pc_in);
	std::vector<pcl17::PointIndices> cluster_indices;
	pcl17::EuclideanClusterExtraction<T> ec;
	ec.setClusterTolerance (tolerance); 
	ec.setMinClusterSize (min_size);
	ec.setMaxClusterSize (max_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pc_in);
	ec.extract(cluster_indices);

	int j=0;

	//clear points from clusters
	clusters->erase(clusters->begin(), clusters->end());

	for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it !=cluster_indices.end (); ++it)
	{
		clusters->push_back(boost::shared_ptr<pcl17::PointCloud<T> > (new pcl17::PointCloud<T>));

		for (std::vector<int>::const_iterator pit =	it->indices.begin(); pit != it->indices.end (); pit++)
		{
			clusters->at(j)->points.push_back(pc_in->points[*pit]); 
		}
		clusters->at(j)->width = clusters->at(j)->points.size();
		clusters->at(j)->height = 1;
		clusters->at(j)->header.frame_id = pc_in->header.frame_id;
		clusters->at(j)->is_dense = true;
		j++;
	}

	return 1;
}


/**
 * @brief 
 *
 * @tparam T
 * @param pc_in
 * @param polygon
 * @param min_distance
 * @param max_distance
 * @param pc_out
 *
 * @return 
 */
	template <typename T>
int extract_polygonal_prism(boost::shared_ptr<pcl17::PointCloud<T> > pc_in, 
		boost::shared_ptr<pcl17::PointCloud<T> > polygon,
		double min_distance, double max_distance,
		boost::shared_ptr<pcl17::PointCloud<T> > pc_out)
{

	pcl17::ExtractPolygonalPrismData<T> epp;                                
	pcl17::ExtractIndices<T> extract1; //Create the extraction object
	pcl17::PointIndices::Ptr indices = pcl17::PointIndices::Ptr(new pcl17::PointIndices);

	//Set epp parameters
	epp.setInputCloud(pc_in);
	epp.setInputPlanarHull(polygon);
	epp.setHeightLimits(min_distance, max_distance); 
	epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
	epp.segment(*indices);

	extract1.setInputCloud(pc_in);
	extract1.setIndices(indices);
	extract1.setNegative(false);
	extract1.filter(*pc_out);


	return 1;
}
