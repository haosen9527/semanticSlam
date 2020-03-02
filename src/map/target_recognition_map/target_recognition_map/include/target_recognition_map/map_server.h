#ifndef MAP_SERVER_H
#define MAP_SERVER_H
/*
 * map_server @haosen
 * Function:    map_save
 *              map_loader
*/
#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <stdio.h>
#include <stdexcept>
#include <ros/ros.h>
#include <cstdio>
#include "ros/console.h"
#include <SDL/SDL_image.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "target_recognition_map_msg/target_recognition_map.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace target_recognition
{
using namespace std;
class mapServer{
public:
    mapServer(std::string mapname,bool savemapsucceed,int thresholdfree,int thresholdoccupied):
        mapName(mapname),saveMapSucceed(savemapsucceed),threshold_free(thresholdfree),threshold_occupied(thresholdoccupied)
    {
            (*mapSub) = mapServerNH.subscribe("map",1,&mapServer::saveMap,this);
    }
    mapServer()
    {
        *mapPub = mapServerNH.advertise<nav_msgs::OccupancyGrid>("map",1,true);
        //loadMap();
    }
    ~mapServer();
    void saveMap(const target_recognition_map_msg::target_recognition_mapConstPtr& map);
    bool loadMap(nav_msgs::OccupancyGrid* map, const char* fname, double res, bool negate,
                 double occ_th, double free_th, double* origin);
public:
    string mapName;
    bool saveMapSucceed;
    int threshold_free;
    int threshold_occupied;

    //ros
    ros::NodeHandle mapServerNH;
    ros::Subscriber* mapSub;
    ros::Publisher* mapPub;
    ros::ServiceServer mapser;
    nav_msgs::OccupancyGrid* mapMsg;
};



}


#endif
