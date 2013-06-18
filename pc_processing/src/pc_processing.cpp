#include <pc_processing/pc_processing.h>

#ifndef _PC_PROCESSING_CPP_
#define _PC_PROCESSING_CPP_
#endif

template bool get_distance_point_to_tf_origin(   tf::TransformListener* p_tf_listener, 
		std::string point_frame_id, 
		std::string tf_frame_id,
		pcl17::PointXYZ pt, 
		ros::Time t,
		double *distance);

template bool get_distance_point_to_tf_origin(   tf::TransformListener* p_tf_listener, 
		std::string point_frame_id, 
		std::string tf_frame_id,
		pcl17::PointXYZRGB pt, 
		ros::Time t,
		double *distance);

template double distance_from_point_to_plane(pcl17::PointXYZ pt, pcl17::ModelCoefficients::Ptr coeff);
template double distance_from_point_to_plane(pcl17::PointXYZRGB pt, pcl17::ModelCoefficients::Ptr coeff);


template int create_line_strip_vizmarker_from_pc(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> >  pc_in,
										visualization_msgs::Marker* marker);

template int create_line_strip_vizmarker_from_pc(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >  pc_in,
										visualization_msgs::Marker* marker);


template int project_point_to_plane(pcl17::PointXYZRGB* ptin, pcl17::ModelCoefficients::Ptr coeff, pcl17::PointXYZRGB* ptout);
template int project_point_to_plane(pcl17::PointXYZ* ptin, pcl17::ModelCoefficients::Ptr coeff, pcl17::PointXYZ* ptout);

template void create_reference_frame_from_plane_and_two_points( pcl17::ModelCoefficients::Ptr plane,
																pcl17::PointXYZRGB *pt1,
																pcl17::PointXYZRGB *pt2,
																tf::Transform* transf);

template void create_reference_frame_from_plane_and_two_points( pcl17::ModelCoefficients::Ptr plane,
																pcl17::PointXYZ *pt1,
																pcl17::PointXYZ *pt2,
																tf::Transform* transf);

template int filter_along_dimension(pcl17::PointCloud<pcl17::PointXYZ>::Ptr pc_in,
								    double threshold,
		 						    std::string dimension,
						   			boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >  pc_out);

template int filter_along_dimension(pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr pc_in,
								    double threshold,
		 						    std::string dimension,
						   			boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> >  pc_out);

		
template int detect_largest_plane(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> >  pc_in,
						 double threshold,
				    	 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> >  pc_out,
						 boost::shared_ptr<pcl17::ModelCoefficients> coefficients,
						 int max_iterations); 

template int detect_largest_plane(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >  pc_in,
						 double threshold,
						 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> >  pc_out,
						 boost::shared_ptr<pcl17::ModelCoefficients> coefficients,
						 int max_iterations); 

template int project_pc_to_plane(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, 
								 boost::shared_ptr<pcl17::ModelCoefficients> coefficients, 
								 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_out);

template int project_pc_to_plane(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, 
								 boost::shared_ptr<pcl17::ModelCoefficients> coefficients, 
								 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_out);


template int compute_convex_hull(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, 
								 pcl17::ModelCoefficients::Ptr coeff,
								 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > ch_out);

template int compute_convex_hull(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, 
								 pcl17::ModelCoefficients::Ptr coeff,
								 boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > ch_out);

template int downsample_pc(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, double dx, double dy, double dz, boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_out);


template int downsample_pc(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, double dx, double dy, double dz, boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_out);

template int get_max_group_from_clustering(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, 
								  double tolerance,
			   					  size_t min_size,
								  size_t max_size, 
								  boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_out);

template int get_max_group_from_clustering(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, 
								  double tolerance,
			   					  size_t min_size,
								  size_t max_size, 
								  boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_out);

template int extract_polygonal_prism(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, 
							boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > polygon,
							double min_distance, double max_distance,
							boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_out);

template int extract_polygonal_prism(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, 
							boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > polygon,
							double min_distance, double max_distance,
							boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_out);

template int get_all_groups_from_clustering(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > pc_in, 
											double tolerance,
											size_t min_size,
											size_t max_size, 
											std::vector< boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > >* clusters);

template int get_all_groups_from_clustering(boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > pc_in, 
											double tolerance,
											size_t min_size,
											size_t max_size, 
											std::vector< boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZ> > >* clusters);

