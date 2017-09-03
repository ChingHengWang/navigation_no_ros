#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

//#include <my_navfn/navfn_no_ros.h>
//#include <cv_debug_header.h>
#include <sys/time.h>
#include <math.h>
//ROS
#include <ros/ros.h>
//X11
#include <thread>
#include <X11/Xlib.h>

using namespace std;
nav_msgs::Odometry robot_odom_; 
//lock
boost::recursive_mutex odom_lock_;

void odomCallback(const nav_msgs::Odometry& odom_in)
{
  odom_lock_.lock();

  robot_odom_.pose.pose.position.x = odom_in.pose.pose.position.x;
  robot_odom_.pose.pose.position.y = odom_in.pose.pose.position.y;
  robot_odom_.pose.pose.position.z = odom_in.pose.pose.position.z;

  robot_odom_.pose.pose.orientation.x = odom_in.pose.pose.orientation.x;
  robot_odom_.pose.pose.orientation.y = odom_in.pose.pose.orientation.y;
  robot_odom_.pose.pose.orientation.z = odom_in.pose.pose.orientation.z;
  robot_odom_.pose.pose.orientation.w = odom_in.pose.pose.orientation.w;
  //cout<<"odomCallback robot_odom_.x " <<robot_odom_.pose.pose.position.x<<endl;
  //cout<<"odomCallback robot_odom_.y " <<robot_odom_.pose.pose.position.y<<endl;
  odom_lock_.unlock();
}

int main(int argc, char **argv)
{
  XInitThreads(); 
/* initial variable */
  ros::init(argc, argv, "test");   
  ros::NodeHandle nh;  
  ros::Subscriber odom_sub = nh.subscribe("/andbot/odom_diffdrive", 10, &odomCallback);
  /* initial costmap */
  costmap_2d::Costmap2DROS* planner_costmap_ros_;
  costmap_2d::Costmap2DROS* local_costmap_ros_;
 
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", &robot_odom_, true);
  local_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", &robot_odom_, false);


  ros::Rate r(10);
/*while loop*/
  while(ros::ok())
  { 
    ros::spinOnce();
    r.sleep();
  }

  


}
