#include <ros/ros.h> 
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "math.h"
#include <iostream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
geometry_msgs::PoseArray cam1_extrinsic;
geometry_msgs::PoseArray cam2_extrinsic;
int cb_i,cb_j=0 ;
void odom1_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose p; 
    Eigen::Vector3d position;
    double norm;
    cam1_extrinsic.header.stamp = ros::Time::now(); 
    cam1_extrinsic.header.frame_id = "map" ;
    p.position.x=msg->pose.pose.position.x;
    p.position.y=msg->pose.pose.position.y;
    p.position.z=msg->pose.pose.position.z;
    p.orientation.x=msg->pose.pose.orientation.x;
    p.orientation.y=msg->pose.pose.orientation.y;
    p.orientation.z=msg->pose.pose.orientation.z;
    p.orientation.w=msg->pose.pose.orientation.w;
    cam1_extrinsic.poses.push_back(p);
    cb_i++;
}
void odom2_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose p; 
    Eigen::Vector3d position;
    double norm;
    cam2_extrinsic.header.stamp = ros::Time::now(); 
    cam2_extrinsic.header.frame_id = "map" ;
    p.position.x=msg->pose.pose.position.x;
    p.position.y=msg->pose.pose.position.y;
    p.position.z=msg->pose.pose.position.z;
    p.orientation.x=msg->pose.pose.orientation.x;
    p.orientation.y=msg->pose.pose.orientation.y;
    p.orientation.z=msg->pose.pose.orientation.z;
    p.orientation.w=msg->pose.pose.orientation.w;
    cam2_extrinsic.poses.push_back(p);
    cb_j++;
}
void yaml_write()
{
    std::ofstream fout ("/home/ncrl/catkin_ws/src/itri_project/config/extrinsic_calibration.yaml");
    YAML::Emitter out(fout);
                out << YAML::BeginMap;
                out<<YAML::Key << "p1_x";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].position.x;
                out<<YAML::Key << "p1_y";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].position.y;
                out<<YAML::Key << "p1_z";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].position.z;
                 out<<YAML::Key << "q1_w";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].orientation.w;
                out<<YAML::Key << "q1_x";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].orientation.x;
                out<<YAML::Key << "q1_y";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].orientation.y;
                out<<YAML::Key << "q1_z";
                out << YAML::Value << cam1_extrinsic.poses[cb_i-1].orientation.z;
                 out<<YAML::Key << "p2_x";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].position.x;
                out<<YAML::Key << "p2_y";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].position.y;
                out<<YAML::Key << "p2_z";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].position.z;
                 out<<YAML::Key << "q2_w";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].orientation.w;
                out<<YAML::Key << "q2_x";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].orientation.x;
                out<<YAML::Key << "q2_y";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].orientation.y;
                out<<YAML::Key << "q2_z";
                out << YAML::Value << cam2_extrinsic.poses[cb_j-1].orientation.z;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagslam_odom");
    ros::NodeHandle n;
    ros::Subscriber odom1_sub = n.subscribe("/tagslam/odom/body_rig1", 1000, odom1_callback);
    ros::Subscriber odom2_sub = n.subscribe("/tagslam/odom/body_rig2", 1000, odom2_callback);
    ros::Rate loop_rate(20);
    ROS_INFO("Start writing the extrisic to yaml.file");
    while(ros::ok()){
        
        if (cam1_extrinsic.poses.size() != 0)
        {
            if (cb_i>30 && cb_j>30)
            {
                std::cout<<"Calibration Done !!"<<std::endl;
                yaml_write();
                break;
            }
        }
        
        ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
