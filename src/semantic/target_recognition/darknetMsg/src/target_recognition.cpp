#include "ros/ros.h"
#include "iostream"
#include "string"
#include "target_recognition_map_msg/target_base_list.h"


using namespace std;

class target_recognition
{
public:
  target_recognition(string target_recogniti_topic):loop_rate(10)
  {
    target_recognition_pub = target_recognition_nh.advertise<target_recognition_map_msg::target_base_list>(target_recogniti_topic,1000);
    init_loop();
  }
  void init_loop()
  {
    while(ros::ok())
    {
      target_recognition_map_msg::target_base_list target_recognition_msg;
      //  ==
      //target_recognition_msg
      target_recognition_pub.publish(target_recognition_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
public:
  ros::NodeHandle target_recognition_nh;
  ros::Publisher target_recognition_pub;
  ros::Rate loop_rate;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_recognition");
  std::string target_recognition_topic = "/semantic/target_recognition_msg";
  ros::param::get("target_recognition_topic",target_recognition_topic);
  target_recognition target_recognition(target_recognition_topic);

  return 0;
}
