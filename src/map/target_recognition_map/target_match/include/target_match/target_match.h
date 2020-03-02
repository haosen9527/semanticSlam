#ifndef TARGET_MATCH_H
#define  TARGET_MATCH_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <target_recognition_map_msg/target_base_list.h>
#include <sensor_msgs/Imu.h>
namespace target_recognition {

class target_match
{
public:
    target_match(std::string darknetopic,std::string imutopic,std::string yamlFilename = ros::package::getPath("target_recognition_map")+"/yaml/example.yaml"):darknetTopic(darknetopic),imuTopic(imutopic),fileName(yamlFilename)
    {
        *darknet = targrtMatchNH.subscribe(darknetTopic,1,&target_match::darkCallback,this);
        *mag = targrtMatchNH.subscribe(imuTopic,1,&target_match::imuCallback,this);
    }
    void darkCallback(const target_recognition_map_msg::target_base_listConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void match_yaml();
public:
    //TOPIC & name
    std::string darknetTopic;
    std::string imuTopic;
    std::string fileName;
    //ros
    ros::NodeHandle targrtMatchNH;
    ros::Subscriber* darknet;
    ros::Subscriber* mag;
    //msg
    target_recognition_map_msg::target_base_listConstPtr recMapMsg;
    sensor_msgs::ImuConstPtr recImuMsg;
};
}
#endif
