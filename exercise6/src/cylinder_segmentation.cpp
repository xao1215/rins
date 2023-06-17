#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/tf.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"

// ros::Publisher pubx;
// ros::Publisher puby;
ros::Publisher pubm;


tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

visualization_msgs::MarkerArray marker_array;
visualization_msgs::MarkerArray parking_array;



void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
	// All the objects needed
	if ( marker_array.markers.size() == 8 ){return;}

	ros::Time time_rec, time_test;
	time_rec = ros::Time::now();

	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	Eigen::Vector4f centroid;

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);

	// Read in the cloud data

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);
	// std::cerr << "PointCloud has: " << cloud_filtered->points.size() << " data points." << std::endl;

	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.015f, 0.015f, 0.015f);
	sor.filter(*cloud_filtered_blob);
	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud);

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	// std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(80);
	seg.setDistanceThreshold(0.07);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients

	seg.segment(*inliers_plane, *coefficients_plane);

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);

	// std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

	pcl::PCLPointCloud2 outcloud_plane;
	pcl::toPCLPointCloud2(*cloud_plane, outcloud_plane);
	// pubx.publish(outcloud_plane);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.005);
	seg.setRadiusLimits(0.10, 0.15);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	try{
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
	}catch(...){
		return;
	}

	// std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;



	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);


	if( cloud_cylinder->points.empty() || cloud_cylinder->points.size() < 300 )
		// std::cerr << "NO CYLINDER" << std::endl;
		return;
	else{
		// std::cerr << "CZLINDER: " << cloud_cylinder->points.size() << " data points." << std::endl;

		pcl::compute3DCentroid(*cloud_cylinder, centroid);
		// std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

		// Create a point in the "camera_rgb_optical_frame"
		geometry_msgs::TransformStamped tss;

		// geometry_msgs::PointStamped point_camera;
		// point_camera.header.frame_id = "camera_rgb_optical_frame";
		// point_camera.header.stamp = ros::Time::now();
		// point_camera.point.x = centroid[0]/2,
		// point_camera.point.y = centroid[1]/2;
		// point_camera.point.z = centroid[2]/2;

		geometry_msgs::PointStamped point_camera_1;
		point_camera_1.header.frame_id = "camera_rgb_optical_frame";
		point_camera_1.header.stamp = ros::Time::now();
		point_camera_1.point.x = centroid[0]/1.5,
		point_camera_1.point.y = centroid[1]/1.5;
		point_camera_1.point.z = centroid[2]/1.5;

		geometry_msgs::PointStamped point_camera_2;
		point_camera_2.header.frame_id = "camera_rgb_optical_frame";
		point_camera_2.header.stamp = ros::Time::now();
		point_camera_2.point.x = centroid[0]/1,
		point_camera_2.point.y = centroid[1]/1;
		point_camera_2.point.z = centroid[2]/1;

		// geometry_msgs::PointStamped point_map;
		// point_map.header.frame_id = "map";
		// point_map.header.stamp = ros::Time::now();

		geometry_msgs::PointStamped point_map_1;
		point_map_1.header.frame_id = "map";
		point_map_1.header.stamp = ros::Time::now();

		geometry_msgs::PointStamped point_map_2;
		point_map_2.header.frame_id = "map";
		point_map_2.header.stamp = ros::Time::now();

		try
		{
			time_test = ros::Time::now();
			tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
			// tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("Transform warning: %s\n", ex.what());
		}

		// tf2::doTransform(point_camera, point_map, tss);
		tf2::doTransform(point_camera_1, point_map_1, tss);
		tf2::doTransform(point_camera_2, point_map_2, tss);



		float r = 0, g = 0, b = 0;
		int cloud_sajz = cloud_cylinder->points.size();
		for(int nIndex = 0; nIndex < cloud_sajz; nIndex++){
				uint32_t nig = *reinterpret_cast<uint32_t*>( &cloud_cylinder->points[nIndex].rgb );
				r += (nig & 16711680) >> 16;
				g += (nig & 65280) >> 8;
				b += (nig & 255);			
		}
		r = (r / (cloud_sajz * 255.0f));
		g =  (g / (cloud_sajz * 255.0f));
		b =  (b / (cloud_sajz * 255.0f));
		

		for( int i = 0; i < marker_array.markers.size(); i++ ){
			int x = marker_array.markers[i].pose.position.x - point_map_2.point.x;
			int y = marker_array.markers[i].pose.position.y - point_map_2.point.y;
			int dist = sqrt(x*x + y*y);
			if( dist < 0.15 ){ 		
        	  	// std::cerr << "alredz there" << std::endl;
				// pcl::PCLPointCloud2 outcloud_cylinder;
				// pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
				// puby.publish(outcloud_cylinder);
				return;
			}
		}

		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "cylinder";
		marker.id = marker_array.markers.size();
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = point_map_2.point.x;
		marker.pose.position.y = point_map_2.point.y;
		marker.pose.position.z = point_map_2.point.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0f;
		marker.lifetime = ros::Duration();
		marker_array.markers.push_back(marker);

		// LAST THING WAS DOING SO THAT FOR EACH SEPAREATE MARKER WE UPDATE

		visualization_msgs::Marker marke;
		marke.header.frame_id = "map";
		marke.header.stamp = ros::Time::now();
		marke.ns = "cube";
		marke.id = marker_array.markers.size();
		marke.type = visualization_msgs::Marker::CUBE;
		marke.action = visualization_msgs::Marker::ADD;
		marke.pose.position.x = point_map_1.point.x;
		marke.pose.position.y = point_map_1.point.y;
		marke.pose.position.z = point_map_1.point.z;
		marke.pose.orientation.x = 0.0;
		marke.pose.orientation.y = 0.0;
		marke.pose.orientation.z = 0.0;
		marke.pose.orientation.w = 1.0;
		marke.scale.x = 0.1;
		marke.scale.y = 0.1;
		marke.scale.z = 0.1;
		marke.color.r = r;
		marke.color.g = g;
		marke.color.b = b;
		marke.color.a = 1.0f;
		marke.lifetime = ros::Duration();
		marker_array.markers.push_back(marke);
		pubm.publish(marker_array);



		// pcl::PCLPointCloud2 outcloud_cylinder;
		// pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
		// puby.publish(outcloud_cylinder);
	}
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "cylinder_segment");
	ros::NodeHandle nh;

	// For transforming between coordinate frames
	tf2_ros::TransformListener tf2_listener(tf2_buffer);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	// pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
	// puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

	pubm = nh.advertise<visualization_msgs::MarkerArray>("detected_cylinders", 100);

	// Spin
	ros::spin();
}