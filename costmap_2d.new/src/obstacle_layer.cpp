#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <string>
#include <Eigen/Core>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
using namespace cv;

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()
{
  obstacle_sub = obstacle_nh.subscribe("/xv11_scan", 10, &ObstacleLayer::laserScanCallback,this);
  //obstacle_sub_2 = obstacle_nh.subscribe("/andbot/odom_diffdrive", 10, &ObstacleLayer::odomCallback,this);
  //obstacle_pub = obstacle_nh.advertise<std_msgs::String>("/my_test", 10);
  obstacle_pub = obstacle_nh.advertise<sensor_msgs::PointCloud>("/observation_cloud", 10);
 

  ROS_INFO("subscribe to xv11\n"); 
  printf("start initialize obstacle layer\n");
  //ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();
  printf("rolling_window %d\n",rolling_window_);
 
  bool track_unknown_space;
  track_unknown_space =  layered_costmap_->isTrackingUnknown();
  printf("track_unknown_space %d\n",track_unknown_space);
 
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;
  
  cout<<"ObstacleLayer::matchSize()"<<endl;
  ObstacleLayer::matchSize();
  cout<<"ObstacleLayer::matchSize() end"<<endl;
  current_ = true;

  //global_frame_ = layered_costmap_->getGlobalFrameID();
  global_frame_ = "";
  double transform_tolerance = g_transform_tolerance ;

  std::string topics_string = "LaserScan";
  // now we need to split the topics based on whitespace which we can use a stringstream for
  printf("start read source in obstacle layer\n");
 
  //std::stringstream ss(topics_string);
  //std::string source;

//  while (ss >> source)
//  {
  printf("source obstacle layer\n");

  // get the parameters for the specific topic
  double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
  std::string topic, sensor_frame, data_type;
  bool inf_is_valid, clearing, marking;

  //source_node.param("topic", topic, source);
  sensor_frame = std::string("");
  observation_keep_time =  0.0;
  expected_update_rate =  0.0;
  data_type = std::string("LaserScan");
  min_obstacle_height =  0.0;
  max_obstacle_height =  2.0;
  inf_is_valid =  true;
  clearing =  true;
  marking =  true;
  printf("source obstacle layer\n");

  if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
  {
    //ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
    throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
  }

  // get the obstacle range for the sensor
  double obstacle_range = g_obstacle_range;

  // get the raytrace range for the sensor
  double raytrace_range = g_raytrace_range;
  printf("source obstacle layer\n");

  // create an observation buffer
  observation_buffers_.push_back(
      boost::shared_ptr < ObservationBuffer
          > (new ObservationBuffer("", observation_keep_time, expected_update_rate, min_obstacle_height,
                                   max_obstacle_height, obstacle_range, raytrace_range,  "",
                                   sensor_frame, transform_tolerance, robot_odom_)));
  printf("source obstacle layer, marking %d\n",marking);

  // check if we'll add this buffer to our marking observation buffers
  cout<<"marking =\n";
  cout<<marking<<endl;
  cout<<"clearing =\n";
  cout<<clearing<<endl;
  if (marking)
    marking_buffers_.push_back(observation_buffers_.back());

  // check if we'll also add this buffer to our clearing observation buffers
  if (clearing)
    clearing_buffers_.push_back(observation_buffers_.back());

  // create a callback for the topic
  if (data_type == "LaserScan"){

    //ros::Subscriber laser_sub = nh_.subscribe("/xv11_scan", 1000, laserScanCallback);
 
//    }// while(ss>source)
        /*
      boost::thread td_LaseScanData( boost::bind(&ObstacleLayer::laserScanCallback, this, observation_buffers_.back()));
      printf("source obstacle layer\n");
    */
    boost::thread ObsLayerCB( boost::bind(&ObstacleLayer::reconfigureCB, this));
  }
}

ObstacleLayer::~ObstacleLayer()
{
}

void ObstacleLayer::reconfigureCB()
{
  enabled_ = g_obstacle_enabled;
  footprint_clearing_enabled_ = g_obstacle_footprint_clearing_enabled;
  max_obstacle_height_ = g_map_max_obstacle_height;
  combination_method_ = g_obstacle_combination_method;
}

//void ObstacleLayer::laserScanCallback(const boost::shared_ptr<ObservationBuffer>& buffer)
/*
void ObstacleLayer::odomCallback(const nav_msgs::Odometry& odom_in)
{
//  std::cout<<"odom in"<<<<std::endl;
//  cout<<"x"<<odom_in.pose.pose.position.x<<endl;
//  cout<<"y"<<odom_in.pose.pose.position.y<<endl;
//  cout<<"z"<<odom_in.pose.pose.position.z<<endl;
  double x = odom_in.pose.pose.position.x;
  double y = odom_in.pose.pose.position.y;
  double z = odom_in.pose.pose.position.z;


  //cout<<"odom in"<<" x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
  odom_lock.lock();
  robot_odom_->pose.pose.position.x = odom_in.pose.pose.position.x;
  robot_odom_->pose.pose.position.y = odom_in.pose.pose.position.y;
  robot_odom_->pose.pose.position.z = odom_in.pose.pose.position.z;

  robot_odom_->pose.pose.orientation.x = odom_in.pose.pose.orientation.x;
  robot_odom_->pose.pose.orientation.y = odom_in.pose.pose.orientation.y;
  robot_odom_->pose.pose.orientation.z = odom_in.pose.pose.orientation.z;
  robot_odom_->pose.pose.orientation.w = odom_in.pose.pose.orientation.w;
  odom_lock.unlock();

  //cout<<"odom_in"<<endl;
  //cout<<odom_in<<endl;
  //cout<<"robot_odom"<<endl;
  //cout<<robot_odom<<endl;
  
}
*/

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScan& raw_scan_in)
{
  /*
  std_msgs::String msg;
  std::stringstream ss;
  ss<<"heelo world";
  msg.data = ss.str();
  obstacle_pub.publish(msg);
  */
  //cout<<"laserScanCallback\n"<<endl;
  boost::shared_ptr<ObservationBuffer>& buffer = observation_buffers_.back();
  //std::string tmp_str = scan_in.header.frame_id;//xv11
  //printf("msg header %s\n",tmp_str.c_str());
  // project the laser into a point cloud

  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan scan_in = raw_scan_in;
  for (size_t i = 0; i < scan_in.ranges.size(); i++)
  {
    float range = scan_in.ranges[ i ];
    if (!std::isfinite(range) && range > 0)
    {
      scan_in.ranges[ i ] = scan_in.range_max - epsilon;
    }
  }

  
  size_t n_pts = scan_in.ranges.size();
  Eigen::ArrayXXd ranges (n_pts, 2);
  Eigen::ArrayXXd output (n_pts, 2);
  Eigen::ArrayXXd co_sine_map_;

  for (size_t i = 0; i < n_pts; ++i)
  {
    ranges (i, 0) = (double) scan_in.ranges[i];
    ranges (i, 1) = (double) scan_in.ranges[i];
  }

  co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
  for (size_t i = 0; i < n_pts; ++i)
  {
    co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
    co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
  }
  output = ranges * co_sine_map_;

  sensor_msgs::PointCloud2 cloud_out;
  cloud_out.header = scan_in.header;
  cloud_out.height = 1;
  cloud_out.width = scan_in.ranges.size();
  cloud_out.fields.resize (3);

  cloud_out.fields[0].name = "x";
  cloud_out.fields[0].offset = 0;
  cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[0].count = 1;

  cloud_out.fields[1].name = "y";
  cloud_out.fields[1].offset = 4;
  cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[1].count = 1;

  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 8;
  cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[2].count = 1;
  int offset = 12;
  cloud_out.point_step = offset;
  cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
  cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
  cloud_out.is_dense = false;


  int field_size = cloud_out.fields.size();
  unsigned int count = 0;
  for (size_t i = 0; i < n_pts; ++i)
  {
    const float range = scan_in.ranges[i];
    if (range >= scan_in.range_min)
    {
      if(true)
      {
        float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];
        // Copy XYZ
        pstep[0] = output (i, 0);
        pstep[1] = output (i, 1);
        pstep[2] = 0;
        ++count; 
      }
    }  
  }
  //pub.publish(cloud_out);


  //read lase point data from naoqi sdk

  // buffer the point cloud  
  buffer->lock();
  buffer->bufferCloud(cloud_out);
  buffer->unlock();
  //cout<<cloud_out<<endl;
  //cout<<"size"<<endl;
  //std::cout<<observation_buffers_.size()<<std::endl;
if(false){
  {
    cout<<"observation buffer: In laserScan Callback"<<endl;
    std::vector<Observation> observations;
    observation_buffers_[0]->lock();
    observation_buffers_[0]->getObservations(observations);
    observation_buffers_[0]->unlock();
    std::vector<Observation>::iterator it;
    for(it=observations.begin();it!=observations.end();it++)
    {
      cout<<"cloud size " <<observations.size()<<endl;
 
      Observation obs = *it;
      pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
      std::cout<<*cloud_ptr<<std::endl;
    }
  }
}
if(false){
  {
    cout<<"marking buffer"<<endl;
    std::vector<Observation> observations;
    marking_buffers_[0]->lock();
    marking_buffers_[0]->getObservations(observations);
    marking_buffers_[0]->unlock();
    std::vector<Observation>::iterator it;
    for(it=observations.begin();it!=observations.end();it++)
    {
      cout<<"cloud size " <<observations.size()<<endl;
 
      Observation obs = *it;
      pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
      std::cout<<*cloud_ptr<<std::endl;
    }
  }
}
 if(false){
  {
    cout<<"clearing buffer"<<endl;
    std::vector<Observation> observations;
    clearing_buffers_[0]->lock();
    clearing_buffers_[0]->getObservations(observations);
    clearing_buffers_[0]->unlock();
    std::vector<Observation>::iterator it;
    for(it=observations.begin();it!=observations.end();it++)
    {
      cout<<"cloud size " <<observations.size()<<endl;
 
      Observation obs = *it;
      pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
      std::cout<<*cloud_ptr<<std::endl;
    }
  }
}
 
  /*
  {
    cout<<"debug buffer1"<<endl;
    std::vector<Observation> observations;
    observation_buffers_[0]->lock();
    observation_buffers_[0]->getObservations(observations);
    observation_buffers_[0]->unlock();
    std::vector<Observation>::iterator it;
    for(it=observations.begin();it!=observations.end();it++)
    {
      Observation obs = *it;
      pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
      std::cout<<*cloud_ptr<<std::endl;
    }
  } 
  */

}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  cout<<name_<<" updateBoudns\n"<<endl;
  

  /*check rolling window*/
  //cout<<"ObstacleLayer::updateBounds\n";
  if(false)
  {
    cout<<"obstacleLayer : observation buffer"<<endl;
    std::vector<Observation> observations;
    observation_buffers_[0]->lock();
    observation_buffers_[0]->getObservations(observations);
    observation_buffers_[0]->unlock();
    cout<<"observation size= \n";
    cout<<observations.size()<<endl;
    std::vector<Observation>::iterator it;
    for(it=observations.begin();it!=observations.end();it++)
    {
      cout<<"cloud size " <<observations.size()<<endl;
 
      Observation obs = *it;
      pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
      std::cout<<*cloud_ptr<<std::endl;
    }
  
  }

  
  //end
  if(name_ == "local_costmap/obstacle_layer")
  {
    cout<<"local_costmap"<<endl;
    cout<<"rooling_window_: "<<rolling_window_<<endl;
    cout<<"robot_x: "<<robot_x<<endl;
    cout<<"robot_y: "<<robot_y<<endl;
  }
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
 
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;
  /*
  cout<<"marking observation_bufers size= \n";
  cout<<observations[0].cloud_->width<<endl;
  */


  /*check observation buffer*/
  /*
  cout<<"observation_bufers observation_keep_time= \n";
  cout<<observation_buffers_[0]->getObservationKeepTime()<<endl;
  cout<<"observation_buifers expected update rate= \n";
  cout<<observation_buffers_[0]->getExpectedUpdateRate()<<endl;
  cout<<"observation_bufers global frame= \n";
  cout<<observation_buffers_[0]->getGlobalFrame()<<endl;
  cout<<"observation_bufers sensor frame= \n";
  cout<<observation_buffers_[0]->getSensorFrame()<<endl;
  cout<<"observation_bufers obstacle range= \n";
  cout<<observation_buffers_[0]->getObstacleRange()<<endl;
  cout<<"observation_bufers raytrace range= \n";
  cout<<observation_buffers_[0]->getRayTraceRange()<<endl;
  */

  /*  
  cout<<"marking observation_bufers size= \n";
  cout<<observations.size()<<endl;
  */
  /*
  if(observations.size()!=0)
  {
    cout<<"marking observation_bufers[0] origin_.x= \n";
    cout<<observations[0].getOriginX()<<endl;
    cout<<"marking observation_bufers[0] origin_.y= \n";
    cout<<observations[0].getOriginY()<<endl;
    cout<<"marking observation_bufers[0] origin_.z= \n";
    cout<<observations[0].getOriginZ()<<endl;

    cout<<"marking observation_bufers[0] raytrace range= \n";
    cout<<observations[0].raytrace_range_<<endl;
    cout<<"marking observation_bufers[0] obstacle range= \n";
    cout<<observations[0].obstacle_range_<<endl;

    pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = observations[0].cloud_;
    std::cout<<*cloud_ptr<<std::endl;
  }
  */

  /*failed */
  /*
*/  //
/*
  geometry_msgs::Point origin_;
  pcl::PointCloud<pcl::PointXYZ>* cloud_;
  double obstacle_range_, raytrace_range_;
*/
 
  
  /* 
  cout<<"marking observations size= \n";
  cout<<observations.size()<<endl;
  cout<<"marking observations keep time= \n";
  cout<<observations.<<endl;


  cout<<"clearing observations size= \n";
  cout<<clearing_observations.size()<<endl;
  */
  
  //end




  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);//相信後面的SENSOR，取聯集最大FREE SPACE
  }


  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

   

    if(false)
    {
      sensor_msgs::PointCloud msg;
      msg.header.frame_id = "odom";
      //msg.points = cloud.points;
      int point_size = cloud.points.size();
      msg.points.resize(point_size);

      for(unsigned int i= 0;i<point_size;i++)
      {
        msg.points[i].x = cloud.points[i].x;
        msg.points[i].y = cloud.points[i].y;
        msg.points[i].z = cloud.points[i].z;
      }
      obstacle_pub.publish(msg);
    } 
    
    
    
    
    //std::cout<<cloud<<endl;
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    int count = 0;
    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      //cout<<"In Cloud "<<"i = " <<i<<endl;
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        //ROS_DEBUG("The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);


      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
      {
        //ROS_DEBUG("The point is too far away");
        continue;
      }
      /*
      cout<<"rolling Window= \n";
      cout<<rolling_window_<<endl;

      cout<<"LETHAL_OBSTACLE index i= "<<i<<endl;
      cout<<"LETHAL_OBSTACLE pz= "<<pz<<endl;
      cout<<"LETHAL_OBSTACLE sq_dist= "<<sq_dist<<endl;
      cout<<"LETHAL_OBSTACLE sq_obstacle_range= "<<sq_obstacle_range<<endl;
      cout<<"LETHAL_OBSTACLE i= "<<i<<endl;
      cout<<"LETHAL_OBSTACLE px= "<<px<<endl;
      cout<<"LETHAL_OBSTACLE py= "<<py<<endl;
      cout<<"LETHAL_OBSTACLE pz= "<<pz<<endl;
      */


      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      /*
      cout<<"LETHAL_OBSTACLE worldToMap(px, py, mx, my)= "<<worldToMap(px, py, mx, my)<<endl;
      cout<<"LETHAL_OBSTACLE mx my= "<<mx<<" "<<my<<endl;
      cout<<"LETHAL_OBSTACLE origin_x origin_y= "<<origin_x_<<" "<<origin_y_<<endl;
      */


      //px py  在世界座標系{w}也是odom座標系下的量測點位置
      //把該點轉回到地圖座標系上
      //origin_x 世界座標系看地圖
      //origin_y 世界座標系看地圖
      if (!worldToMap(px, py, mx, my))//有推導筆記
      {
        //ROS_DEBUG("Computing map coords failed");
        continue;
      }
      
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
      count++;
    }

    //double ratio = double(count) /double( cloud.points.size());
    //cout <<"ratio = "<<ratio<<endl;


    /*Display costmap*/
    /*TO DO*/
    //cout<<"name_: "<<name_<<endl;

    if(false){
    if(name_ == "local_costmap/obstacle_layer")
    {
      cv::Mat M_tmp=cv::Mat(size_y_,size_x_,CV_8UC1);
      memcpy(M_tmp.data,costmap_,size_x_*size_y_*sizeof(int8_t));
      imshow("local_costmap obstacleLayer costmap in updatebounds", M_tmp);
      waitKey(10);
    }

    if(name_ == "global_costmap/obstacle_layer")
    {
      cv::Mat M_tmp=cv::Mat(size_y_,size_x_,CV_8UC1);
      memcpy(M_tmp.data,costmap_,size_x_*size_y_*sizeof(int8_t));
      imshow("global_costmap obstacleLayer costmap in updatebounds", M_tmp);
      waitKey(10);
    }
    }
    /*Display Costmap*/



  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{

  /*Display costmap*/
  /*TO DO*/
  if(false)
  {
  if(name_ == "local_costmap/obstacle_layer")
  {
    cv::Mat M_tmp=cv::Mat(size_y_,size_x_,CV_8UC1);
    memcpy(M_tmp.data,costmap_,size_x_*size_y_*sizeof(int8_t));
    imshow("local obstacleLayer costmap in updateCost", M_tmp);
    waitKey(100);
  }
  if(name_ == "global_costmap/obstacle_layer")
  {
    cv::Mat M_tmp=cv::Mat(size_y_,size_x_,CV_8UC1);
    memcpy(M_tmp.data,costmap_,size_x_*size_y_*sizeof(int8_t));
    imshow("global obstacleLayer costmap in updateCost", M_tmp);
    waitKey(100);
  }
  }
  
  
  /*Display Costmap*/


  if (!enabled_)
    return;

  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) // combination_method = 0
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  //cout<<"In getMarkingObservations\n";
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    //cout<<"i=\n";
    //cout<<i<<endl;
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  /*
  {
    Observation obs = marking_observations[0];
    pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
    //std::cout<<*cloud_ptr<<std::endl;
  }
  */


  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
    //ROS_WARN_THROTTLE(
    //    1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
    //    ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x)
    {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  /*for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }*/

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void ObstacleLayer::deactivate()
{
  /*for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }*/
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
