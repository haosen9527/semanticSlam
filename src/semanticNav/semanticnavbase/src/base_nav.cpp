#include "iostream"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <string>
#include <ros/param.h>
#include <tf/transform_datatypes.h>

using namespace std;

class baseNav
{
public:
  baseNav(string imutopic):imuTopic(imutopic)
  {
    imuMsg = nh.subscribe(imuTopic,100,&baseNav::imuCallback,this);
  }
  ~baseNav(){}
  void imuCallback(const sensor_msgs::Imu &imumsg);
public:
  ros::NodeHandle nh;
  ros::Subscriber imuMsg;
  ros::Time wakeup;
  string imuTopic;
  double roll,pitch,yaw;
};
 void baseNav::imuCallback(const sensor_msgs::Imu& imumsg)
 {
   //quaternion-->RPY
   tf::Quaternion quat;
   tf::quaternionMsgToTF(imumsg.orientation,quat);
   tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
 }



int main(int argc,char** argv)
{
  ros::init(argc,argv,"base_nav");
  string imuTopic = "/camera/imu";
  ros::param::get("imuTopic",imuTopic);
  baseNav base_nav(imuTopic);
  ros::spin();
  return 0;
}

