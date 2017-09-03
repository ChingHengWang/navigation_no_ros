/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/
#include <base_local_planner_test/odometry_helper_ros.h>
#include <base_local_planner_test/goal_functions.h>

namespace base_local_planner_test {

OdometryHelperRos::OdometryHelperRos(std::string odom_topic) {
  setOdomTopic( odom_topic );
}

void OdometryHelperRos::odomCallback(const arc::Odometry msg) {
   // ROS_INFO_ONCE("odom received!");
    printf("odom received!\n");

  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twistWithCovariance.twist.linear.x = msg.twistWithCovariance.twist.linear.x;
  base_odom_.twistWithCovariance.twist.linear.y = msg.twistWithCovariance.twist.linear.y;
  base_odom_.twistWithCovariance.twist.angular.z = msg.twistWithCovariance.twist.angular.z;
  base_odom_.header.frame_id = msg.header.frame_id;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

//copy over the odometry information
void OdometryHelperRos::getOdom(arc::Odometry& base_odom) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}


void OdometryHelperRos::getRobotVel(arc::PoseStamped& robot_vel) {
  // Set current velocities from odometry
   arc::Twist global_vel;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    global_vel.linear.x = base_odom_.twistWithCovariance.twist.linear.x;
    global_vel.linear.y = base_odom_.twistWithCovariance.twist.linear.y;
    global_vel.angular.z = base_odom_.twistWithCovariance.twist.angular.z;

    //robot_vel.header.frame_id_ = base_odom_.child_frame_id;
  }
  //robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  //robot_vel.stamp_ = ros::Time();
  robot_vel.pose.position.x=global_vel.linear.x;
  robot_vel.pose.position.y=global_vel.linear.y;
  robot_vel.pose.position.z=0;
  arc::Vector3 euler;
  euler.x=0.0;
  euler.y=0.0;
  euler.z=global_vel.angular.z;
  robot_vel.pose.orientation=base_local_planner_test::getQuaternion(euler);
}

void OdometryHelperRos::setOdomTopic(std::string odom_topic)
{
  if( odom_topic != odom_topic_ )
  {
    odom_topic_ = odom_topic;

    if( odom_topic_ != "" )
    {
      //ros::NodeHandle gn;
      //odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &OdometryHelperRos::odomCallback, this, _1 ));
    }
    else
    {
      //odom_sub_.shutdown();
    }
  }
}

} /* namespace base_local_planner_test */
