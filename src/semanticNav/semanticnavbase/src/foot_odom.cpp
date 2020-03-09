#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include <bear_msg/Joy_Data.h>
#include <geometry_msgs/Twist.h>

class footOdom
{
public:
  footOdom():current_time(0),last_time(0)
  {
   x =y = th = 0.0;
   vx = vy = vth = 0.0;
  }
  ~footOdom()
  {

  }
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void compute_model();
public:
  ros::Publisher odomPub;
  ros::Subscriber cmd_vel;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;

  double x,y,th;
  double vx ,vy,vth;
};

void footOdom::compute_model()
{
  current_time = ros::Time::now();
  if(last_time.toSec() != 0)
  {
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    std::cout<<"delta_T:"<<dt<<std::endl;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odomPub.publish(odom);
  }
  last_time = current_time;
}

void footOdom::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  current_time = ros::Time::now();
  vx = (msg->linear.x)*0.9;
  vy = (msg->linear.y)*0.9 ;
  vth = (msg->angular.z)*0.8;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_odom");
  ros::NodeHandle nh;

  footOdom odomRoute;
  odomRoute.cmd_vel = nh.subscribe("bear_cmd", 10,&footOdom::cmdVelCallback,&odomRoute );
  odomRoute.odomPub = nh.advertise<nav_msgs::Odometry>("odom",10);

  ros::Rate sleep_rate(1);
  while(ros::ok())
  {
    sleep_rate.sleep();
    odomRoute.compute_model();
    ros::spinOnce();
  }

  return 0;
}
