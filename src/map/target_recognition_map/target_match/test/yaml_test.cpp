#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
using namespace std;

int main()
{
    YAML::Node test = YAML::LoadFile(ros::package::getPath("target_recognition_map")+"/yaml/example.yaml");
    for(int i=0;i<test.size();i++)
        cout<<"INFO: "<<test[i]["classInfos"].size()<<endl;
    return 0;
}
