#include <costmap_2d/observation_buffer.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace tf;

namespace costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     //double raytrace_range, TransformListener& tf, string global_frame,
                                     double raytrace_range, string global_frame,
                                     string sensor_frame, double tf_tolerance, nav_msgs::Odometry* robot_odom) :
//    observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
//    last_updated_(ros::Time::now()), 
    global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance),robot_odom_(robot_odom)
{
    observation_keep_time_ = observation_keep_time;
    expected_update_rate_ = expected_update_rate;
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
//  printf("ros::Time::now in observation buffer\n");

//  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  /*
  if (!tf_.waitForTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_),
                            ros::Duration(0.01), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }
  */

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      //origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      //tf_.transformPoint(new_global_frame, origin, origin);
      //....!!!!!!!!use pcl to implement this function
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      //pcl_ros::transformPointCloud(new_global_frame, *obs.cloud_, *obs.cloud_, tf_);
      //....!!!!!!!!use pcl to implement this function
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  try
  {
    //cout<<"try bufferCloue form PoiintCloud2 to pcl"<<endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    // Actually convert the PointCloud2 message into a type we can reason about
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    bufferCloud(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
    return;
  }
  
}

void ObservationBuffer::bufferCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  double px = robot_odom_->pose.pose.position.x;
  double py = robot_odom_->pose.pose.position.y;
  double pz = robot_odom_->pose.pose.position.z;
  double qx = robot_odom_->pose.pose.orientation.x;
  double qy = robot_odom_->pose.pose.orientation.y;
  double qz = robot_odom_->pose.pose.orientation.z;
  double qw = robot_odom_->pose.pose.orientation.w;


  //cout<<"odom read"<<" x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
  //cout<<*robot_odom_<<endl;    
  //cout<<"buffer cloude int to pcl pointxyz"<<endl;
  //Stamped < tf::Vector3 > global_origin;
  Eigen::Vector4f global_origin_h;
  Eigen::Vector3f global_origin;
 
  //cout<<cloud<<endl;
  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    //Stamped < tf::Vector3 > local_origin(tf::Vector3(0, 0, 0),
    //                        pcl_conversions::fromPCL(cloud.header).stamp, origin_frame);
/*
    tf::Quaternion q(qx,qy,qz,qw);
    tf::Vector3 p(px,py,pz);
    tf::Transform odom_t_sensor(q,p);
*/

   
    //cout<<"local_origin = \n"<<local_origin<<endl;
    //cout<<"global_origin = \n"<<global_origin<<endl;

    //tf_.waitForTiransform(global_frame_, local_origin.frame_id_, local_origin.stamp_, ros::Duration(0.5));
    //tf_.transformPoint(global_frame_, local_origin, global_origin);
    //............!!!!!use pcl to transform pointcloud 

    Eigen::Vector4f local_origin_h(0,0,0,1);
    Eigen::Vector3f local_origin = local_origin_h.head<3>();
    Eigen::Matrix4f odom_T_sensor = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation = Eigen::Quaternionf(qw,qx,qy,qz).toRotationMatrix();
    Eigen::Vector3f translation(px,py,pz);
    odom_T_sensor.block(0,0,3,3) = rotation;
    odom_T_sensor.block(0,3,3,1) = translation;
    //cout<<odom_T_sensor<<endl;
    global_origin_h = odom_T_sensor*local_origin_h;
    global_origin = global_origin_h.head<3>();
 
    
    
    /*
    observation_list_.front().origin_.x = global_origin.getX();
    observation_list_.front().origin_.y = global_origin.getY();
    observation_list_.front().origin_.z = global_origin.getZ();
    */
    observation_list_.front().origin_.x = global_origin(0);
    observation_list_.front().origin_.y = global_origin(1);
    observation_list_.front().origin_.z = global_origin(2);
 
    
    
    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;
  
    //cout <<"raytrace_range_ =\n"<<raytrace_range_<<endl;
    //cout <<"obstacle_range_ =\n"<<obstacle_range_<<endl;


      
    pcl::PointCloud < pcl::PointXYZ > global_frame_cloud;

    // transform the point cloud
    //pcl_ros::transformPointCloud(global_frame_, cloud, global_frame_cloud, tf_);
    //............!!!!!use pcl to transform pointcloud 

    pcl::transformPointCloud(cloud, global_frame_cloud, odom_T_sensor);
    /*
    cout<<"local cloud\n";
    cout<<cloud<<endl;  
    cout<<"global cloud\n";
    cout<<global_frame_cloud<<endl; 
    */     
    global_frame_cloud.header.stamp = cloud.header.stamp;
    //cout<<global_frame_cloud.header.stamp<<endl;
    // now we need to remove observations from the cloud that are below or above our height thresholds
    pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(observation_list_.front().cloud_);
    unsigned int cloud_size = global_frame_cloud.points.size();
    observation_cloud.points.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
      if (global_frame_cloud.points[i].z <= max_obstacle_height_
          && global_frame_cloud.points[i].z >= min_obstacle_height_)
      {
        observation_cloud.points[point_count++] = global_frame_cloud.points[i];
      }
    }
    //cout<<"observation_cloud\n";
    //cout<<observation_cloud<<endl;
    // resize the cloud for the number of legal points
    observation_cloud.points.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;

  // if the update was successful, we want to update the last updated time
  //  printf("ros::Time::now in observation buffer2\n");

  //  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
  /*
  cout<<"robot x=\n"<<robot_odom_->pose.pose.position.x<<endl;
  cout<<"robot y=\n"<<robot_odom_->pose.pose.position.y<<endl;
  cout<<"robot z=\n"<<robot_odom_->pose.pose.position.z<<endl;
  */



}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  //cout<<"In getObservations\n";
  // first... let's make sure that we don't have any stale observations
  //purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
  //Observation obs = observations[0];
  //pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = obs.cloud_;
  //std::cout<<*cloud_ptr<<std::endl;
  /*
  cout<<"observation_list size "<<observation_list_.size()<<endl;
  cout<<observation_list_.front().origin_.x<<endl;
  cout<<observation_list_.front().origin_.y<<endl;
  cout<<observation_list_.front().origin_.z<<endl;
  cout<<observation_list_.front().raytrace_range_<<endl;
  cout<<observation_list_.front().obstacle_range_<<endl;
  */
  //pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = observation_list_.front().cloud_;
  //pcl::PointCloud<pcl::PointXYZ> cloud = *cloud_ptr; 
  //Observation obs = observation_list_.front();
  /*
  pcl::PointCloud<pcl::PointXYZ>* cloud_ptr = observation_list_.front().cloud_;
  cout<<cloud_ptr->width<<endl;
  cout<<cloud_ptr->height<<endl;
 */
  //cout<<*cloud_ptr<<endl;
 
  //list<Observation>::iterator obs_it_2;
  //obs_it_2 = observation_list_.begin();
}

void ObservationBuffer::purgeStaleObservations()
{
  //cout<<"purgeStaleObservations keep time "<<observation_keep_time_<<endl;
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == 0.0)
            //ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      //ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
    /*ZACH
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    */
    }
  }

}

bool ObservationBuffer::isCurrent() const
{
/*    
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  //printf("ros::Time::now in observation 3\n");

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    printf("The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
  */
}

void ObservationBuffer::resetLastUpdated()
{
  //last_updated_ = ros::Time::now();
}
}  // namespace costmap_2d

