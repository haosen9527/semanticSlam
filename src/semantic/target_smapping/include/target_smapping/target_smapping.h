#ifndef TARGET_SMAPPING_H
#define TARGET_SMAPPING_H

#include "boost/thread.hpp"
#include "ros/ros.h"
#include "ros/param.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/OccupancyGrid.h"
#include "target_recognition_map_msg/target_base_list.h"

class target_smapping
{
public:
  target_smapping():transform_thread_(NULL),tf_delay_(1.0)
  {
    depth_topic = "/camera/depth";
    odom_topic = "/odom";
    target_recognition_topic = "/target_recognition_topic";
    imu_topic = "/imu";

    odom_frame_id = "odom_frame";
    camera_frame_id = "camera_depth";
    map_frame_id = "map";

    transform_publish_period_ = 0.05;

    sensors_init();
  }
  void sensors_init();
  //sensor callback
  void depth_camera_callback(const sensor_msgs::Image &img_msg);
  void odom_callback(const nav_msgs::Odometry &odom_mag);
  void target_recognition_callback(const target_recognition_map_msg::target_base_list &target_recognition_msg);
  void imu_callback(const sensor_msgs::Imu &imu_msg);
  void publishLoop(double transform_publish_period);
  void publishTransform();
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

  std::string odom_frame_id;
  std::string camera_frame_id;
  std::string map_frame_id;

  //tf thread
  boost::thread* transform_thread_;
  tf::Transform map_to_odom_;
  boost::mutex map_to_odom_mutex_;
  double transform_publish_period_;
  double tf_delay_;

  //tf
  tf::TransformBroadcaster* tfB_;
};
#endif // TARGET_SMAPPING_H
