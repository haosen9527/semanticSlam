#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "iostream"
#include "string"
#include "target_recognition_map_msg/target_base_list.h"


using namespace std;

class target_recognition
{
public:
  target_recognition(string target_recogniti_topic,string imu_topic):loop_rate(5)
  {
    target_recognition_pub = target_recognition_nh.advertise<target_recognition_map_msg::target_base_list>(target_recogniti_topic,1000);
    imu  = target_recognition_nh.subscribe(imu_topic,100,&target_recognition::imuCallback,this);
    init_loop();
  }
  void init_loop()
  {
    while(ros::ok())
    {
      target_recognition_map_msg::target_base_list target_recognition_msg_list;
      //msg reset
      target_recognition_map_msg::target_base target_recognition_msg;
      target_recognition_msg.className = "person";
      target_recognition_msg.position.x = 50;
      target_recognition_msg.position.y = 60;
//      target_recognition_msg.orientation = orientation;

      target_recognition_msg_list.classInfos.push_back(target_recognition_msg);
      //target_recognition_msg
      target_recognition_msg_list.orientation = orientation;
      target_recognition_pub.publish(target_recognition_msg_list);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  void imuCallback(const sensor_msgs::Imu& imu_msg)
  {
    orientation = imu_msg.orientation;
  }
public:
  ros::NodeHandle target_recognition_nh;
  ros::Publisher target_recognition_pub;
  ros::Subscriber imu;
  geometry_msgs::Quaternion orientation;
  ros::Rate loop_rate;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_recognition");
  std::string target_recognition_topic = "/semantic/target_recognition_msg";
  ros::param::get("target_recognition_topic",target_recognition_topic);
  std::string imu_topic = "/imu";
  ros::param::get("imu",imu_topic);
  target_recognition target_recognition(target_recognition_topic,imu_topic);

  return 0;
}
