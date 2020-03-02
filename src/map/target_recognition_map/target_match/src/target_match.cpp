#include "target_match/target_match.h"


namespace target_recognition {

void target_match::darkCallback(const target_recognition_map_msg::target_base_listConstPtr &msg)
{
    recMapMsg = msg;
}
void target_match::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    recImuMsg = msg;
}
void target_match::match_yaml()
{
    YAML::Node mapTrack = YAML::LoadFile(fileName);
    for(int i =0;i<mapTrack.size();i++)
    {
//        if(mapTrack[i]["classInfos"].size() == recMapMsg->classInfos.size())
//        {
//            for(int j=0;j<mapTrack[i]["classInfos"].size();j++)
//            {
//                mapTrack[i]["classInfos"][recMapMsg->classInfos[j]];
//            }
//        }
    }
}


}//namespace
