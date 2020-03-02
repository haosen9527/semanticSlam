#include "target_smapping/target_smapping.h"

void target_smapping::sensors_init()
{
  ros::param::get("depth_topic",depth_topic);
  ros::param::get("odom_topic",odom_topic);
  ros::param::get("target_recognition_topic",target_recognition_topic);

  depth_camera = target_smapping_nh.subscribe<sensor_msgs::Image>(depth_topic,10,&target_smapping::depth_camera_callback,this);
  odom = target_smapping_nh.subscribe<nav_msgs::Odometry>(odom_topic,10,&target_smapping::odom_callback,this);
  target_recognition = target_smapping_nh.subscribe<target_recognition_map_msg::target_base_list>(target_recognition_topic,10,&target_smapping::target_recognition_callback,this);
  odom = target_smapping_nh.subscribe<sensor_msgs::Imu>(odom_topic,10,&target_smapping::imu_callback,this);
}

void target_smapping::depth_camera_callback(const sensor_msgs::Image img_msg)
{

}
void target_smapping::odom_callback(const nav_msgs::Odometry odom_mag)
{

}
void target_smapping::target_recognition_callback(const target_recognition_map_msg::target_base_list target_recognition_msg)
{

}
void target_smapping::imu_callback(const sensor_msgs::Imu imu_msg)
{

}
