#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

int m_count=0;

using namespace std;

namespace costmap_2d
{
Costmap2DROS::Costmap2DROS(std::string name, nav_msgs::Odometry* robot_odom, bool isGlobalCostmap) :
    layered_costmap_(NULL), name_(name), stop_updates_(false), initialized_(true), stopped_(false),
    robot_stopped_(false), map_update_thread_(NULL), robot_odom_(robot_odom)//, last_publish_(0)
{
  //printf("costmap2dros start constructor\n");

  bool rolling_window,track_unknown_space,always_send_full_costmap;
  // check if we want a rolling window version of the costmap
  // global costmap
  // rolling_window false
  // track_unknown_space false
  if(isGlobalCostmap)
  {
    rolling_window = false;
    track_unknown_space = false;
  }
  // local costmap
  // rolling_window true
  // track_unknown_space false
  else
  {
    rolling_window = true;
    track_unknown_space = false;
  }
  
  /*
  bool rolling_window = g_rolling_window; // true
  bool track_unknown_space = false;  //false or true?
  bool always_send_full_costmap;
  */
  layered_costmap_ = new LayeredCostmap("", rolling_window, track_unknown_space);
  //printf("costmap2dros start constructor\n");


  if( isGlobalCostmap )
  {
    //static costmap layer only for global costmap
    std::string pname = "static_layer";
    boost::shared_ptr<StaticLayer> plugin(new StaticLayer());        //layer class
    layered_costmap_->addPlugin(plugin);
    plugin->initialize(layered_costmap_, name + "/" + pname, robot_odom_);
    //printf("costmap2dros finish static layer\n");

  }
  //obstacle layer for both global and local costmap

   
  std::string pname_obst = "obstacle_layer";
  boost::shared_ptr<ObstacleLayer> plugin_obst(new ObstacleLayer());        //layer class
  layered_costmap_->addPlugin(plugin_obst);
  plugin_obst->initialize(layered_costmap_, name + "/" + pname_obst, robot_odom_);
  printf("costmap2dros finish obstacle layer\n");
  

  //Inflation layer for both global and local costmap
  std::string pname_infla = "inflation_layer";
  boost::shared_ptr<InflationLayer> plugin_infla(new InflationLayer());        //layer class
  layered_costmap_->addPlugin(plugin_infla);
  plugin_infla->initialize(layered_costmap_, name + "/" + pname_infla, robot_odom_);
  //printf("costmap2dros finish inflation layer\n");
 

  //set foot print from radius params or you can choose ploygon footprint
  setUnpaddedRobotFootprint(makeFootprintFromRadius( g_robot_radius));

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a timer to check if the robot is moving
  robot_stopped_ = false;
  double pose_update_frequency = g_pose_update_frequency;
  double time_sleep = 1./pose_update_frequency * 1000*1000;
  //printf("costmap2dros start constructor3\n");

 
  //boost::thread poseCB( boost::bind( &Costmap2DROS::movementCB, this, time_sleep) );
  boost::thread MapUpdateCB( boost::bind( &Costmap2DROS::reconfigureCB, this) );
  
  //poseCB.join();
  MapUpdateCB.join();
 
}
/*
void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}
*/
Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }

  delete layered_costmap_;
}

void Costmap2DROS::reconfigureCB()
{
  cout<<"Costmap2dros reconfigureCB\n";
  cout<<"g_map_update_frequency = \n";
  cout<<g_map_update_frequency<<endl;

  transform_tolerance_ = g_transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_us = (1.0/g_map_update_frequency)*1000*1000;

  cout<<"map_update_us = 1.0/g_map_update_frequency*1000*1000 = \n";
  cout<<map_update_us<<endl;


  // find size parameters
  double map_width_meters = g_map_width, map_height_meters = g_map_height, resolution = g_map_resolution, origin_x =
             g_origin_x,
         origin_y = g_origin_y;
  cout<<"map_width_meters = \n";
  cout<<map_width_meters<<endl;

  cout<<"map_height_meters = \n";
  cout<<map_height_meters<<endl;

  cout<<"resolution = \n";
  cout<<resolution<<endl;

  cout<<"origin_x = \n";
  cout<<origin_x<<endl;
  cout<<"origin_y = \n";
  cout<<origin_y<<endl;
 
  cout<<"name = \n";
  cout<<name_<<endl;
  cout<<"global_frame_ = \n";
  cout<<g_global_frame<<endl;

  cout<<"robot_base_frame_ = \n";
  cout<<g_robot_base_frame<<endl;
 
  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_us));
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

//void Costmap2DROS::movementCB(const ros::TimerEvent &event)
void Costmap2DROS::movementCB( double time_sleep)
{
  // don't allow configuration to happen while this check occurs
   boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  tf::Stamped < tf::Pose > new_pose;

  if (!getRobotPose(new_pose))
  {
    //ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling pose reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3
      && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3)
  {
    old_pose_ = new_pose;
    robot_stopped_ = true;
  }
  else
  {
    old_pose_ = new_pose;
    robot_stopped_ = false;
  }

  usleep( time_sleep ); //100ms
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{

  cout<<"mapUpdateThread\n";
  cout<<"frequency = \n";
  cout<<frequency<<endl;


  //printf("costmap2dros mapupdateloop start\n");


  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  //ros::NodeHandle nh;
  //ros::Rate r(frequency);
  while ( !map_update_thread_shutdown_)
  {
    
    //struct timeval start, end;
    //double start_t, end_t, t_diff;
    //gettimeofday(&start, NULL);

    updateMap();
 
    //gettimeofday(&end, NULL);
    //start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    //end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    //t_diff = end_t - start_t;
    //ROS_DEBUG("Map update time: %.9f", t_diff);
    /*if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }*/

    usleep(frequency);
    //r.sleep();
    // make sure to sleep for the remainder of our cycle time
    //if (r.cycleTime() > ros::Duration(1 / frequency))
    //  ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
    //           r.cycleTime().toSec());
  }
}

void Costmap2DROS::updateMap()
{
  if (!stop_updates_)
  {

    // get global pose
    tf::Stamped < tf::Pose > pose;
    if (getRobotPose (pose))
    {
      double x = pose.getOrigin().x(),
             y = pose.getOrigin().y(),
             yaw = tf::getYaw(pose.getRotation());

      layered_costmap_->updateMap(x, y, yaw);

      /*Display costmap*/
      /*TO DO*/
      if(name_ == "global_costmap" && true)
      {
        Costmap2D* costmap_tmp = layered_costmap_->getCostmap();
        unsigned int size_x = costmap_tmp->getSizeInCellsX();
        unsigned int size_y = costmap_tmp->getSizeInCellsY();

        unsigned char* costmap_array = costmap_tmp->getCharMap();
        cv::Mat M_tmp=cv::Mat(size_y,size_x,CV_8UC1);
        memcpy(M_tmp.data,costmap_array,size_x*size_y*sizeof(int8_t));
        imshow("global_costmap costmap2dros updateMap", M_tmp);
        waitKey(100);
      }
      /*Display Costmap*/

      if(name_ == "local_costmap"&&( m_count%10==0))
      {
        Costmap2D* costmap_tmp = layered_costmap_->getCostmap();
        unsigned int size_x = costmap_tmp->getSizeInCellsX();
        unsigned int size_y = costmap_tmp->getSizeInCellsY();

        unsigned char* costmap_array = costmap_tmp->getCharMap();
        cv::Mat M_tmp=cv::Mat(size_y,size_x,CV_8UC1);
        memcpy(M_tmp.data,costmap_array,size_x*size_y*sizeof(int8_t));
        imshow("local_costmap costmap2dros updateMap", M_tmp);
        waitKey(100);
        m_count=0;
      }
      m_count++;
      /*Display Costmap*/



      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;

      //printf("ros::Time::now in updateMap\n");
      //footprint.header.stamp = ros::Time::now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      //footprint_pub_.publish(footprint);

      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  //ros::Rate r(100.0);
  while ( !initialized_ )
    usleep( 100000 );
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  //ros::Rate r(100.0);
  while (!initialized_)
    usleep( 100000 );
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
  global_pose.setIdentity();
  //tf::Stamped < tf::Pose > robot_pose;
  //robot_pose.setIdentity();
  //robot_pose.frame_id_ = robot_base_frame_;
  //robot_pose.stamp_ = ros::Time();
  //printf("ros::Time::now in getRobotPose\n");

  //ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  double px = robot_odom_->pose.pose.position.x;
  double py = robot_odom_->pose.pose.position.y;
  double pz = robot_odom_->pose.pose.position.z;
  double qx = robot_odom_->pose.pose.orientation.x;
  double qy = robot_odom_->pose.pose.orientation.y;
  double qz = robot_odom_->pose.pose.orientation.z;
  double qw = robot_odom_->pose.pose.orientation.w;

  tf::Vector3 pos(px,py,pz);
  global_pose.setOrigin(pos);
  tf::Quaternion rot(qx,qy,qz,qw);
  global_pose.setRotation(rot);
  

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  tf::Stamped<tf::Pose> global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf::getYaw(global_pose.getRotation());
  transformFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                     padded_footprint_, oriented_footprint);
}

}  // namespace costmap_2d
