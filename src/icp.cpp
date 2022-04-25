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
#include <pcl/registration/icp.h>
#include <chrono>
#include <pcl/filters/passthrough.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr output1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output2_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
Eigen::Matrix4d transformation;
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
	//convert pointcloud2 to pointXYZRGB
    pcl::fromPCLPointCloud2(cloud_filtered,*output1_cloud);
    //remove z >2.5 pointcloud
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(output1_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,2.5);
    pass.filter(*output1_cloud);
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
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(output2_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,2.3);
    pass.filter(*output2_cloud);
}


void f2f_icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2)
{
    pcl::PointCloud<pcl::PointXYZRGB> aligned_f2f;
    //pcl ICP function
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> f2f_ICP;
    //f2f matching 
    f2f_ICP.setInputSource(pointcloud2);
    f2f_ICP.setInputTarget(pointcloud1);
    f2f_ICP.align(aligned_f2f);
    transformation = f2f_ICP.getFinalTransformation().cast<double>();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "merge the pointcloud by ICP");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	ros::Subscriber cloud1_sub = nh.subscribe("/camera1/depth_registered/points", 1, pointcloud1_callback);
	ros::Subscriber cloud2_sub = nh.subscribe("/camera2/depth_registered/points", 1, pointcloud2_callback);
	ROS_INFO("calculate T by ICP method ");

	while (ros::ok()){
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            f2f_icp(output1_cloud,output2_cloud);
            std::chrono::steady_clock::time_point t2= std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
            std::cout<<"ICP cost "<<time_used.count()<<"  seconds ."<<std::endl;
            std::cout<<"Transformation ="<<std::endl<< transformation <<std::endl;
			ros::spinOnce();
			loop_rate.sleep();
	}
	return 0; 
}
