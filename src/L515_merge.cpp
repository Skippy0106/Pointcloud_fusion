#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/tf.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr output1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output2_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 output_msg;
Eigen::Matrix4d T1,T2;
Eigen::Vector3d p1,p2;

void pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
	//convert sensor_msg to pcl::pointcloud2
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	//using voxel filter 
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*input_msg, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor; 
  sor.setInputCloud (cloudPtr);  
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered);
	//convert pointcloud2 to pointXYZ
  pcl::fromPCLPointCloud2(cloud_filtered,*output1_cloud);
  output_msg.header = input_msg->header;
}

void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
	//convert sensor_msg to pcl::pointcloud2
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	//using voxel filter 
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*input_msg, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor; 
  sor.setInputCloud (cloudPtr);  
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered);
	//convert pointcloud2 to pointXYZ
  pcl::fromPCLPointCloud2(cloud_filtered,*output2_cloud);
}

void pose1_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{	
	double px,py,pz,qx,qy,qz,qw;
	qx=input->pose.orientation.x;
	qy=input->pose.orientation.y;
	qz=input->pose.orientation.z;
	qw=input->pose.orientation.w;
	px=input->pose.position.x;
	py=input->pose.position.y;
	pz=input->pose.position.z;
	T1<<1-2*(qy*qy+qz*qz),  2*(qx*qy-qw*qz),  2*(qw*qy+qx*qz),px,
			  2*(qx*qy+qw*qz),1-2*(qx*qx+qz*qz),  2*(qz*qy-qx*qw),py,
			  2*(qx*qz-qw*qz),  2*(qy*qz+qw*qx),1-2*(qx*qx+qy*qy),pz,
			  0,0,0,1;

	p1<<px,pz,pz;
}			

void pose2_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{	
	double px,py,pz,qx,qy,qz,qw;
	qx=input->pose.orientation.x;
	qy=input->pose.orientation.y;
	qz=input->pose.orientation.z;
	qw=input->pose.orientation.w;
	px=input->pose.position.x;
	py=input->pose.position.y;
	pz=input->pose.position.z;
	T2<<1-2*(qy*qy+qz*qz),  2*(qx*qy-qw*qz),  2*(qw*qy+qx*qz),px,
			  2*(qx*qy+qw*qz),1-2*(qx*qx+qz*qz),  2*(qz*qy-qx*qw),py,
			  2*(qx*qz-qw*qz),  2*(qy*qz+qw*qx),1-2*(qx*qx+qy*qy),pz,
			  0,0,0,1;
	p2<<px,pz,pz;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointcloud fusion");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	ros::Subscriber cloud1_sub = nh.subscribe("/camera1/depth_registered/points", 5, pointcloud1_callback);
	ros::Subscriber cloud2_sub = nh.subscribe("/camera2/depth_registered/points", 5, pointcloud2_callback);
	//ros::Subscriber pose1_sub = nh.subscribe("/vrpn_client_node/MAV1/pose1", 1, pose1_callback);
	//ros::Subscriber pose2_sub = nh.subscribe("/vrpn_client_node/MAV1/pose2", 1, pose2_callback);
	ros::Publisher merge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merge_pointcloud", 10);
	Eigen::Matrix4d T;
	/*
	T << 0.9114,-0.0927,-0.4008,1.2348,
			  0.1034, 0.9946,  0.0051,0.0715,
			  0.3982,-0.04607,0.9161,0.1135,
			  0,0,0,1;
			  */
	T << 0.9989,0.0457,0.0085,0.0509,
			  -0.0459, 0.9987,  0.0190,-0.0176,
			  0.0076,-0.0194,0.9997,-0.1415,
			  0,0,0,1;
	ROS_INFO("Start pointcloud merge");

	while (ros::ok()){
			//tranform the pointcloud and merge
			pcl::transformPointCloud (*output2_cloud, *transformed_cloud2,T);
			*merge_pointcloud=*output1_cloud+*transformed_cloud2;
			pcl::toROSMsg(*merge_pointcloud, output_msg);
			merge_cloud_pub.publish(output_msg);
			ros::spinOnce();
			loop_rate.sleep();
	}
	return 0; 
}
