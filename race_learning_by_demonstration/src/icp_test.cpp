
#include <ros/ros.h>
#include <iostream>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/registration/icp.h>

bool received_obj1=false;
pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_obj_to_track;
pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_objects_on_table;
pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_aligned;
ros::Publisher* p_pub_pc1;
ros::Publisher* p_pub_pc2;

void callback_obj1_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
{
	if (received_obj1==false)
	{
		ROS_INFO("Received obj1 Point Cloud. Processing callback."); 
		pcl17::fromROSMsg(*pcmsg_in, *cloud_obj_to_track);
		received_obj1 = true;
	}
}

void callback_objects_on_table_received(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
{
	if (received_obj1==true)
	{
		ROS_INFO("Received objects on table. Commencing ICP registration."); 
		pcl17::fromROSMsg(*pcmsg_in, *cloud_objects_on_table);

		pcl17::IterativeClosestPoint<pcl17::PointXYZ, pcl17::PointXYZ> icp;
		icp.setInputCloud(cloud_obj_to_track);
		icp.setInputTarget(cloud_objects_on_table);
		//icp.setUseReciprocalCorrespondences(true);
		//se::registration::DefaultConvergenceCriteria
		//< Scalar >::Ptr

		//pcl17::registration::DefaultConvergenceCriteria
			//< Scalar >::Ptr

		//(icp.getConvergenceCriteria())->setUseReciprocalCorrespondences(true);

		//Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (0.1);
		//// Set the maximum number of iterations (criterion 1)
		//icp.setMaximumIterations (100);
		//// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (1e-10);
		//// Set the euclidean distance difference epsilon (criterion 3)
		//icp.setEuclideanFitnessEpsilon (0.001);



		icp.align(*cloud_aligned);
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;


		sensor_msgs::PointCloud2 msg; 
		pcl17::toROSMsg(*cloud_aligned,msg); 	
		p_pub_pc1->publish(msg);

		pcl17::toROSMsg(*cloud_obj_to_track,msg); 	
		p_pub_pc2->publish(msg);

		cloud_obj_to_track = cloud_aligned;

	}
}


int main (int argc, char** argv)
{

	ros::init(argc, argv, "learning_by_demonstration"); // Initialize ROS coms
	ros::NodeHandle n("~"); //The node handle

	cloud_obj_to_track = (pcl17::PointCloud<pcl17::PointXYZ>::Ptr) new pcl17::PointCloud<pcl17::PointXYZ>;
	cloud_objects_on_table = (pcl17::PointCloud<pcl17::PointXYZ>::Ptr) new pcl17::PointCloud<pcl17::PointXYZ>;
	cloud_aligned = (pcl17::PointCloud<pcl17::PointXYZ>::Ptr) new pcl17::PointCloud<pcl17::PointXYZ>;

	ros::Subscriber sub_pc = n.subscribe ("/pc_out/obj1", 1, callback_obj1_received);
	ros::Subscriber sub_pc1 = n.subscribe ("/pc_out/objects_on_table", 1, callback_objects_on_table_received);

	ros::Publisher pub_pc1 = n.advertise<sensor_msgs::PointCloud2>("/pc_out/aligned", 1); p_pub_pc1 = &pub_pc1;
	ros::Publisher pub_pc2 = n.advertise<sensor_msgs::PointCloud2>("/pc_out/object_to_track", 1); p_pub_pc2 = &pub_pc2;

	ros::Time t = ros::Time::now();
	//Start program
	ros::Rate loop_rate(10);
	ros::spin();



	//// Fill in the CloudIn data
	//cloud_in->width    = 300000;
	//cloud_in->height   = 1;
	//cloud_in->is_dense = false;
	//cloud_in->points.resize (cloud_in->width * cloud_in->height);
	//for (size_t i = 0; i < cloud_in->points.size (); ++i)
	//{
		//cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		//cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		//cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	//}
	//std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
		//<< std::endl;
	//for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
		//cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
			//cloud_in->points[i].z << std::endl;
	//*cloud_out = *cloud_in;
	//std::cout << "size:" << cloud_out->points.size() << std::endl;
	//for (size_t i = 0; i < cloud_in->points.size (); ++i)
		//cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	//std::cout << "Transformed " << cloud_in->points.size () << " data points:"
		//<< std::endl;
	//for (size_t i = 0; i < cloud_out->points.size (); ++i)
		//std::cout << "    " << cloud_out->points[i].x << " " <<
			//cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	//pcl17::IterativeClosestPoint<pcl17::PointXYZ, pcl17::PointXYZ> icp;
	//icp.setInputCloud(cloud_in);
	//icp.setInputTarget(cloud_out);
	//pcl17::PointCloud<pcl17::PointXYZ> Final;
	//icp.align(Final);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}
