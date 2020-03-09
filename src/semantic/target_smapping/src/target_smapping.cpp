#include "target_smapping/target_smapping.h"

void target_smapping::sensors_init()
{
  ros::param::get("depth_topic",depth_topic);
  ros::param::get("odom_topic",odom_topic);
  ros::param::get("target_recognition_topic",target_recognition_topic);

  ros::param::get("odom_frame_id",odom_frame_id);
  ros::param::get("camera_frame_id",camera_frame_id);
  ros::param::get("map_frame_id",map_frame_id);

  depth_camera = target_smapping_nh.subscribe(depth_topic,10,&target_smapping::depth_camera_callback,this);
  odom = target_smapping_nh.subscribe(odom_topic,10,&target_smapping::odom_callback,this);
  target_recognition = target_smapping_nh.subscribe(target_recognition_topic,10,&target_smapping::target_recognition_callback,this);
  odom = target_smapping_nh.subscribe(odom_topic,10,&target_smapping::imu_callback,this);

  transform_thread_ = new boost::thread(boost::bind(&target_smapping::publishLoop, this, transform_publish_period_));

}

void target_smapping::depth_camera_callback(const sensor_msgs::Image& img_msg)
{

}
void target_smapping::odom_callback(const nav_msgs::Odometry& odom_mag)
{

}
void target_smapping::target_recognition_callback(const target_recognition_map_msg::target_base_list& target_recognition_msg)
{

}
void target_smapping::imu_callback(const sensor_msgs::Imu& imu_msg)
{

}
void target_smapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}
void target_smapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_id, odom_frame_id));
  map_to_odom_mutex_.unlock();
}

