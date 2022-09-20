#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr output1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output2_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 output_msg;
void pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
	//convert sensor_msg to pcl::pointcloud2
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl_conversions::toPCL(*input_msg, *cloud);
	pcl::fromPCLPointCloud2(*cloud,*output1_cloud);
	}

void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
	//convert sensor_msg to pcl::pointcloud2
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl_conversions::toPCL(*input_msg, *cloud);
	pcl::fromPCLPointCloud2(*cloud,*output2_cloud);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointcloud fusion");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	ros::Subscriber cloud1_sub = nh.subscribe("/camera1/depth/color/points", 5, pointcloud1_callback);
	ros::Subscriber cloud2_sub = nh.subscribe("/camera2/depth/color/points", 5, pointcloud2_callback);
	ros::Publisher merge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merge_pointcloud", 10);
	output_msg.header.frame_id="map";
	output_msg.header.stamp = ros::Time::now(); 
	double p1_x , p1_y , p1_z , p2_x , p2_y , p2_z;
	double q1_w , q1_x , q1_y , q1_z , q2_w , q2_x , q2_y , q2_z;
	nh.getParam("p1_x" , p1_x);
	nh.getParam("p1_y" , p1_y);
	nh.getParam("p1_z" , p1_z);
	nh.getParam("p2_x" , p2_x);
	nh.getParam("p2_y" , p2_y);
	nh.getParam("p2_z" , p2_z);
	nh.getParam("q1_x" , q1_x);
	nh.getParam("q1_y" , q1_y);
	nh.getParam("q1_z" , q1_z);
	nh.getParam("q1_w" , q1_w);
	nh.getParam("q2_x" , q2_x);
	nh.getParam("q2_y" , q2_y);
	nh.getParam("q2_z" , q2_z);
	nh.getParam("q2_w" , q2_w);
	Eigen::Quaterniond q_1(q1_w , q1_x , q1_y , q1_z);
	Eigen::Quaterniond q_2(q2_w , q2_x , q2_y , q2_z);
	Eigen::Vector3d t_1(p1_x,p1_y,p1_z);
	Eigen::Vector3d t_2(p2_x,p2_y,p2_z);
	Eigen::Matrix3d R_1 =  q_1.toRotationMatrix();
	Eigen::Matrix3d R_2 =  q_2.toRotationMatrix();
	Eigen::Matrix4d T_1,T_2,T;
	T_1<< R_1(0,0),R_1(0,1),R_1(0,2), t_1(0) ,
				R_1(1,0),R_1(1,1),R_1(1,2), t_1(1)  ,
   				R_1(2,0),R_1(2,1),R_1(2,2), t_1(2) ,
				0               ,0              ,0              ,1; 
	T_2<< R_2(0,0),R_2(0,1),R_2(0,2), t_2(0)  ,
				R_2(1,0),R_2(1,1),R_2(1,2), t_2(1)  ,
   				R_2(2,0),R_2(2,1),R_2(2,2), t_2(2)   ,
				0               ,0              ,0              ,1; 
	T=T_1*T_2;
	ROS_INFO("Start pointcloud merge");
	while (ros::ok()){
			//tranform the pointcloud and merge
			pcl::transformPointCloud (*output1_cloud, *transformed_cloud1,T_1);
			pcl::transformPointCloud (*output2_cloud, *transformed_cloud2,T_2);
			*merge_pointcloud=*transformed_cloud1+*transformed_cloud2;
			pcl::toROSMsg(*merge_pointcloud, output_msg);
			merge_cloud_pub.publish(output_msg);
			ros::spinOnce();
			loop_rate.sleep();
	}
	return 0; 
}
