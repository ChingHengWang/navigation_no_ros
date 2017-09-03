#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP
//#include <costmap_2d/header.h>
#include <costmap_2d/costmap_settings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Quaternion.h>
#include <stdio.h>
using namespace cv;
using namespace tf;
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace costmap_2d
{
    class MapLoader
    {
    public:
        MapLoader(std::string map_name, nav_msgs::OccupancyGrid& map)
        {
          unsigned char* pixels;
          unsigned char* p;
          unsigned char value;
          unsigned int i,j;
          int rowstride;
          int color_sum;
          double occ = -1;
           
          double res = g_resolution;
          int negate = g_negate;
          double occ_th = g_occupied_thresh;
          double free_th = g_free_thresh;
          double origin[3] = {0};
          origin[0] = g_origin_x;
          origin[1] = g_origin_y;
          origin[2] = g_origin_z;

          std::cout<<"map file name is "<<map_name<<endl;
          Mat map_img = imread(map_name, -1);
          if( map_img.empty() )
          {
            printf("Load map file failed!\n");
            return;
          }

          //std::cout<<"we are going to store the map to rgb format!"<<endl;
          //cv::imwrite("map.rgb", map_img);

          map.info.width = map_img.cols;
          map.info.height = map_img.rows;
          map.info.resolution = res;
          map.info.origin.position.x = *(origin);
          map.info.origin.position.y = *(origin+1);
          map.info.origin.position.z = *(origin+2);  
          Quaternion q;
          q.setRPY(0,0, *(origin+2));
          map.info.origin.orientation.x = q.x();
          map.info.origin.orientation.y = q.y();
          map.info.origin.orientation.z = q.z();
          map.info.origin.orientation.w = q.w();
        
          map.data.resize(map.info.width*map.info.height);
        
          // Get values that we'll need to iterate through the pixels
          rowstride = map_img.cols;
          pixels = (unsigned char*)(map_img.data);
        
          for(j = 0; j < map.info.height; j++)
          {
            for(i =0; i < map.info.width; i++)
            {
              // Compute mean of RGB for this pixel
              //p = pixels + j*rowstride + i;
              p = pixels + (map.info.height - j)*rowstride + i;
              // If negate is true, we consider blacker pixels free, and whiter
              // pixels free.  Otherwise, it's vice versa.
              occ = (255 - *p) / 255.0;
              
              if(occ > occ_th){
                value = +100;
              }
              else if(occ < free_th){
                value = 0;
              }
              else {
                //value = -1;
                value = 0; // TEST by ZACH
              }
              map.data[MAP_IDX(map.info.width,i,j)] = value;
            }
          }
          printf("Load map file finish!\n");

        };// end constructor


    };//end class

}//end namespace


#endif