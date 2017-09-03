//#include <costmap_2d/layered_costmap.h>
#include <costmap_2d_test/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
//#include <costmap_2d/map_loader.h>


using namespace std;

namespace costmap_2d_test
{

 Costmap2DROS:: Costmap2DROS() :
    costmap2d_(NULL), stop_updates_(false), initialized_(true), stopped_(true),
    robot_stopped_(false)
{
  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  std::string map;
  global_frame_=map;
  rolling_window=false;
  track_unknown_space=false;
  always_send_full_costmap=false;
  costmap2d_ = new Costmap2D_Test();
}

Costmap2DROS::~Costmap2DROS()
{
  delete costmap2d_;
}

void Costmap2DROS::getRobotP(double& x, double& y, double& z)
{
    x=0.0;
    y=0.2;
    z=0.0;
}

void Costmap2DROS::start()
{
 // getRobotP(robot_x,robot_y, robot_z);
  costmap2d_->updateMap();
}

//void Costmap2D_TestROS::start(costmap_2d::ObstacleLayer obstacleLayer)
//{
//  obstacleLayer.activate(Header::OccupancyGrid map);
//}
/*
void Costmap2D_TestROS::stop()
{
  stop_updates_ = true;
  staticlayer->deactivate();
  initialized_ = false;
  stopped_ = true;
}
*/
/*
void Costmap2D_TestROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}
*/

/*
void Costmap2D_TestROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
}
*/
/*
void Costmap2D_TestROS::resetLayers()
{
  Costmap2D_Test* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  staticlayer->reset();
}
*/
}  // namespace costmap_2d

int main ( int argc, char **argv )
{
    /*MapLoader ml("/home/lixiaopeng/nav_work/costmap_2d/map/TenByTen.yaml");
    OccupancyGrid new_map;
    ml.load(new_map);
    */
    costmap_2d_test::Costmap2D_Test costmap2d_;
    //costmap2d_.incomingMap();
    printf("ni hao\n");
    //costmap2d_->updateMap();
   // unsigned int X;
   // X=costmap2d_.getSizeInCellsX();
  //  printf("X=%d\n",X);
   // costmap2d_.mapUpdateLoop();
    costmap2d_.updateMap();
     unsigned int mx=0;
    unsigned int my=0;
    unsigned char cost=254;
    costmap2d_.setCost( mx,  my,  cost);
    unsigned int index = 0;
    unsigned int X;
    unsigned int Y;
    unsigned char* master;
    X=costmap2d_.getSizeInCellsX();
    Y=costmap2d_.getSizeInCellsY();
   // printf("X=%d Y=%d\n",X,Y);
    master=costmap2d_.getCharMap();
    for (unsigned int i = 0; i <X; ++i)
     {
         for (unsigned int j = 0; j <Y; ++j)
         {
             //printf("master[index]=%d\n",master[index]);
             ++index;
         }
     }

   return 0;
}







