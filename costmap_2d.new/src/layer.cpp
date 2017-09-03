
#include "costmap_2d/layer.h"

namespace costmap_2d
{

Layer::Layer()
  : layered_costmap_(NULL)
  , current_(false)
  , enabled_(false)
  , name_()
  , robot_odom_(NULL)  
//  , tf_(NULL)
{}

//void Layer::initialize(LayeredCostmap* parent, std::string name, tf::TransformListener *tf)
void Layer::initialize(LayeredCostmap* parent, std::string name, nav_msgs::Odometry *robot_odom)
{
  layered_costmap_ = parent;
  name_ = name;
  //tf_ = tf;
  //robot_odom_ = new nav_msgs::Odometry();
  robot_odom_ = robot_odom;
  onInitialize();
}

const std::vector<geometry_msgs::Point>& Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

}  // end namespace costmap_2d
