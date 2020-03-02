#include "ros/ros.h"
#include "ros/package.h"
#include "string"
#include "std_msgs/String.h"
#include "yaml-cpp/yaml.h"
#include "nav_core/base_global_planner.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class voiceToNav
{
public:
  voiceToNav(string voiceTopic):ac("move_base",true)
  {
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    voiceSub = voiceNH.subscribe(voiceTopic,100,&voiceToNav::voiceCmd,this);
  }
  ~voiceToNav(){}
  void voiceCmd(const std_msgs::String& voice_cmd);
  int location();
public:
  MoveBaseClient ac;
  ros::NodeHandle voiceNH;
  ros::Subscriber voiceSub;
  move_base_msgs::MoveBaseGoal goal;
  struct pose_{
    double x;
    double y;
    double z;
    double w;
  }pose;
};

void voiceToNav::voiceCmd(const std_msgs::String& voice_cmd)
{
  ROS_INFO("I heard: [%s]", voice_cmd.data.c_str());

  YAML::Node location_pose = YAML::LoadFile(ros::package::getPath("voice_interface")+"/yaml/pose.yaml");
  for(int i=0;i<location_pose.size();i++)
  {
    if(location_pose[i]["name"].as<string>() == voice_cmd.data)
    {
      location_pose[i]["pose"]["x"] = pose.x;
      location_pose[i]["pose"]["y"] = pose.y;
      location_pose[i]["pose"]["z"] = pose.z;
      location_pose[i]["pose"]["w"] = pose.w;
      break;
    }
  }
  location();

}
int voiceToNav::location()
{
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pose.x;
  goal.target_pose.pose.position.y = pose.y;
  goal.target_pose.pose.position.z = pose.z;
  goal.target_pose.pose.orientation.w = pose.w;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base moved %s meter forward",goal.target_pose.pose.position.x);
  else
    ROS_INFO("The base failed to move for some reason");
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voiceToNav");
  std::string voiceTopic = "/bear/voice/cmd";
  ros::param::get("voiceTopic",voiceTopic);
  voiceToNav voice_interface(voiceTopic);
  ros::spin();
  return 0;
}
