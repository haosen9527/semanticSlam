#ifndef TARGET_SMAPPING_H
#define TARGET_SMAPPING_H

#include "ros/ros.h"
#include "ros/param.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/OccupancyGrid.h"
#include "target_recognition_map_msg/target_base_list.h"

class target_smapping
{
public:
  target_smapping()
  {
    depth_topic = "/camera/depth";
    odom_topic = "/odom";
    target_recognition_topic = "/target_recognition_topic";
    imu_topic = "/imu";

    sensors_init();
  }
  void sensors_init();
  //sensor callback
  void depth_camera_callback(const sensor_msgs::Image img_msg);
  void odom_callback(const nav_msgs::Odometry odom_mag);
  void target_recognition_callback(const target_recognition_map_msg::target_base_list target_recognition_msg);
  void imu_callback(const sensor_msgs::Imu imu_msg);
public:
  ros::NodeHandle target_smapping_nh;
  ros::Subscriber depth_camera;
  ros::Subscriber odom;
  ros::Subscriber target_recognition;
  ros::Subscriber imu;

  std::string depth_topic;
  std::string odom_topic;
  std::string target_recognition_topic;
  std::string imu_topic;


};
#endif // TARGET_SMAPPING_H
