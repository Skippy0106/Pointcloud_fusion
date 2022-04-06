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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


using namespace sensor_msgs;
using namespace message_filters;
pcl::PointCloud<pcl::PointXYZ>cloud1,cloud2;
Eigen::Vector4d q1,q2;
Eigen::Vector3d t1,t2;
void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &  pointcloud_1,const sensor_msgs::PointCloud2::ConstPtr & pointcloud_2)
{
  pcl::PCLPointCloud2 pcl_pc1,pcl_pc2;
  pcl_conversions::toPCL(*pointcloud_1,pcl_pc1);
  pcl_conversions::toPCL(*pointcloud_2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc1,*temp_cloud1);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud2);
  pcl::fromROSMsg(*pointcloud_1,cloud1);
  pcl::fromROSMsg(*pointcloud_2,cloud2);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pointcloud_pose_1,const geometry_msgs::PoseStamped::ConstPtr& pointcloud_pose_2)
{
  q1 << pointcloud_pose_1->pose.orientation.x,pointcloud_pose_1->pose.orientation.y,pointcloud_pose_1->pose.orientation.z,pointcloud_pose_1->pose.orientation.w;
  q2 << pointcloud_pose_2->pose.orientation.x,pointcloud_pose_2->pose.orientation.y,pointcloud_pose_2->pose.orientation.z,pointcloud_pose_2->pose.orientation.w;
  t1 << pointcloud_pose_1->pose.position.x,pointcloud_pose_1->pose.position.y,pointcloud_pose_1->pose.position.z;
  t2 << pointcloud_pose_2->pose.position.x,pointcloud_pose_2->pose.position.y,pointcloud_pose_2->pose.position.z;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "pointcloud fusion");
ros::NodeHandle nh;
pcl::PointCloud<pcl::PointXYZ>  merge_cloud;
Eigen::Vector4d q2_1;
Eigen::Vector3d t2_1;
Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
ros::Publisher merge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fusion_pointcloud", 1000);
message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_1(nh, "/camera1/depth_registered/points", 1);
message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_2(nh, "/camera2/depth_registered/points", 1);
message_filters::Subscriber<geometry_msgs::PoseStamped> pointcloud_pose_1(nh, "/vrpn_client_node/RigidBody1/pose1", 1);
message_filters::Subscriber<geometry_msgs::PoseStamped> pointcloud_pose_2(nh, "/vrpn_client_node/RigidBody1/pose2", 1);
message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_pointcloud(pointcloud_1, pointcloud_2, 10);
message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> sync_pose(pointcloud_pose_1, pointcloud_pose_2, 10);
sync_pointcloud.registerCallback(boost::bind(&pointcloud_callback, _1, _2));
pcl::PointCloud<pcl::PointXYZ>transformed_cloud2;
pcl::transformPointCloud (cloud2, transformed_cloud2, transform);
merge_cloud=cloud1+ transformed_cloud2;
sensor_msgs::PointCloud2 pointcloud_merge;
pcl::toROSMsg(merge_cloud,pointcloud_merge);

ros::spin();
return 0; 
}
