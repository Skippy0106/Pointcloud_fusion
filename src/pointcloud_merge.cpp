#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher g_cloud_pub;
sensor_msgs::PointCloud2 output_msg;
int g_frame_num = 0;

void callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
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
  pcl::fromPCLPointCloud2(cloud_filtered,*output_cloud);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Static;
  Static.setInputCloud(output_cloud);    
  Static.setMeanK(50); 
  Static.setStddevMulThresh(0.5);
  Static.filter(*output_cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
   // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // 距离阈值 单位m
  seg.setDistanceThreshold (0.15);
  seg.setInputCloud (output_cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (output_cloud);
  extract.setIndices (inliers);
  extract.filter (*output_cloud);
  extract.setNegative (true);
   extract.filter (*output_cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "callback");
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Subscriber cloud_sub = nh.subscribe("/camera2/depth_registered/points",2 , callback);
  g_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output_points", 10);
  while (ros::ok()){
  pcl::toROSMsg( *output_cloud, output_msg);
  g_cloud_pub.publish(output_msg);
  ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}
