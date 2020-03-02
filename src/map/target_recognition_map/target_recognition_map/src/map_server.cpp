#include <target_recognition_map/map_server.h>

namespace target_recognition {

void mapServer::saveMap(const target_recognition_map_msg::target_recognition_mapConstPtr &map)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map->info.width,
             map->info.height,
             map->info.resolution);
    std::string mapdatafile = mapName + ".pgm";

    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
      ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
      return;
    }

    fprintf(out, "P5\n# CREATOR: map_server.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
      for(unsigned int x = 0; x < map->info.width; x++) {
        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
        if (map->data[i] >= 0 && map->data[i] <= threshold_free) { // [0,free)
          fputc(254, out);
        } else if (map->data[i] >= threshold_occupied) { // (occ,255]
          fputc(000, out);
        } else { //occ [0.25,0.65]
          fputc(205, out);
        }
      }
    }

    fclose(out);
    std::string mapmetadatafile = mapName + ".yaml";
    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w
    ));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml);

    std::string trackTargetFile = mapName+"Target"+".yaml";
    ROS_INFO("Writing trackTargetFile-map  data to %s", trackTargetFile.c_str());
    FILE* target_yaml = fopen(trackTargetFile.c_str(),"a");
//    ofstream target_yaml(trackTargetFile);
    std::string fprint_string = "-\n    seq: %d\n    mag: [%f,%f,%f]\n    classInfos:\n";
    std::string classInfos = "      %s: %f\n";
    fprintf(target_yaml,fprint_string.c_str(),
            map->header.seq,yaw,pitch,roll);
    for(int i=0;i<map->classInfos.size();i++)
        fprintf(target_yaml,classInfos.c_str(),
                map->classInfos[i].className,map->classInfos[i].position);
    fclose(target_yaml);

    ROS_INFO("Done\n");
    saveMapSucceed = true;
}
bool mapServer::loadMap(nav_msgs::OccupancyGrid* map, const char *fname, double res, bool negate, double occ_th, double free_th, double *origin)
{
    SDL_Surface* img;

    unsigned char* pixels;
    unsigned char* p;
    unsigned char value;
    int rowstride, n_channels, avg_channels;
    unsigned int i,j;
    int k;
    double occ;
    int alpha;
    int color_sum;
    double color_avg;

    // Load the image using SDL.  If we get NULL back, the image load failed.
    if(!(img = IMG_Load(fname)))
    {
      std::string errmsg = std::string("failed to open image file \"") +
              std::string(fname) + std::string("\": ") + IMG_GetError();
      throw std::runtime_error(errmsg);
    }

    // Copy the image data into the map structure
    map->info.width = img->w;
    map->info.height = img->h;
    map->info.resolution = res;
    map->info.origin.position.x = *(origin);
    map->info.origin.position.y = *(origin+1);
    map->info.origin.position.z = 0.0;
//    btQuaternion q;

    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromRollPitchYaw(0,0,*(origin+2));//返回四元数
    // setEulerZYX(yaw, pitch, roll)
    //q.setEulerZYX(*(origin+2), 0, 0);
    map->info.origin.orientation.x = q.x;
    map->info.origin.orientation.y = q.y;
    map->info.origin.orientation.z = q.z;
    map->info.origin.orientation.w = q.w;

    // Allocate space to hold the data
    map->data.resize(map->info.width * map->info.height);

    // Get values that we'll need to iterate through the pixels
    rowstride = img->pitch;
    n_channels = img->format->BytesPerPixel;

    // NOTE: Trinary mode still overrides here to preserve existing behavior.
    // Alpha will be averaged in with color channels when using trinary mode.
    if (!img->format->Amask)
      avg_channels = n_channels;
    else
      avg_channels = n_channels - 1;

    // Copy pixel data into the map structure
    pixels = (unsigned char*)(img->pixels);
    for(j = 0; j < map->info.height; j++)
    {
      for (i = 0; i < map->info.width; i++)
      {
        // Compute mean of RGB for this pixel
        p = pixels + j*rowstride + i*n_channels;
        color_sum = 0;
        for(k=0;k<avg_channels;k++)
          color_sum += *(p + (k));
        color_avg = color_sum / (double)avg_channels;

        if (n_channels == 1)
            alpha = 1;
        else
            alpha = *(p+n_channels-1);

        if(negate)
          color_avg = 255 - color_avg;

        // If negate is true, we consider blacker pixels free, and whiter
        // pixels occupied.  Otherwise, it's vice versa.
        occ = (255 - color_avg) / 255.0;

        // Apply thresholds to RGB means to determine occupancy values for
        // map.  Note that we invert the graphics-ordering of the pixels to
        // produce a map with cell (0,0) in the lower-left corner.
        if(occ > occ_th)
          value = +100;
        else if(occ < free_th)
          value = 0;
        else {
          double ratio = (occ - free_th) / (occ_th - free_th);
          value = 99 * ratio;
        }

        map->data[MAP_IDX(map->info.width,i,map->info.height - j - 1)] = value;
      }
    }

    SDL_FreeSurface(img);

}

}//namespace
