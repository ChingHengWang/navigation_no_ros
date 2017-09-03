/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <base_local_planner_test/goal_functions.h>

namespace base_local_planner_test {

  double getGoalPositionDistance(const arc::PoseStamped& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
  }

  double getYaw(const arc::Quaternion orientation)
  {
	  double yaw;
	  float x=orientation.x;
	  float y=orientation.y;
	  float z=orientation.z;
	  float w=orientation.w;

	  float alpha = atan2( 2*(w*x+y*z), 1- 2*(x*x+y*y) )*180.0/M_PI;
	  float beta = asin( 2*(w*y - z*x) )*180.0/M_PI;
	  float gama = atan2( 2*(w*z + x*y), 1-2*(y*y+z*z) )*180.0/M_PI;
	  yaw = (double)gama;

	  return yaw;
  }

  arc::Quaternion getQuaternion(const arc::Vector3 euler )
  {
	  arc::Quaternion rotation;

	  float cx=cos(euler.x/2);
	  float sx=sin(euler.x/2);
	  float cy=cos(euler.y/2);
	  float sy=sin(euler.y/2);
	  float cz=cos(euler.z/2);
	  float sz=sin(euler.z/2);

	  rotation.x=0.0;
	  rotation.y=0.0;
	  rotation.z=cx*cy*cz + sx*sy*sz;
	  rotation.w=cx*cy*sz - sx*sy*cz;

	  return rotation;
  }

  double getGoalOrientationAngleDifference(const arc::PoseStamped& global_pose, double goal_th) {
    double yaw =getYaw(global_pose.pose.orientation);
    return angles::shortest_angular_distance(yaw, goal_th);
  }

static void RT(const Eigen::Matrix3f& R1, const Eigen::Vector3f& T1, const Eigen::Matrix3f& R2, const Eigen::Vector3f& T2
		  , Eigen::Matrix3f& R, Eigen::Vector3f& T )
{
	R = R1*R2;
	T = R2*T1 + T2;
}

static void QT(const Eigen::Quaternionf& Q1,  const Eigen::Vector3f& T1, const Eigen::Quaternionf& Q2, const Eigen::Vector3f& T2
		  ,Eigen::Quaternionf& Q, Eigen::Vector3f& T)
{
	Eigen::Matrix3f R;
	RT( Q1.toRotationMatrix(), T1, Q2.toRotationMatrix(), T2, R, T);
	Q = R;
}

  void prunePlan(const arc::PoseStamped& global_pose, std::vector<arc::PoseStamped>& plan, std::vector<arc::PoseStamped>& global_plan){
   // ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<arc::PoseStamped>::iterator it = plan.begin();
    std::vector<arc::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const arc::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.pose.position.x - w.pose.position.x;
      double y_diff = global_pose.pose.position.y - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1){
       // ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        printf("Nearest waypoint to: %f, %f, %f, %f\n", global_pose.pose.position.x, global_pose.pose.position.y, w.pose.position.x,  w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  bool transformGlobalPlan( const std::vector<arc::PoseStamped>& global_plan,
         const arc::PoseStamped& global_pose,
         const costmap_2d_test::Costmap2D_Test& costmap,
         const std::string& global_frame,
         std::vector<arc::PoseStamped>& transformed_plan){
           transformed_plan.clear();

         if (global_plan.empty()) {
           printf("Received plan with zero length\n");
           return false;
    }
         printf("Received plan \n");
         int n=global_plan.size();
         printf("n=%d\n",n);
         for(int i =0;i<n;i++)
         {
        	 printf("gx=%f gy=%f\n",global_plan[i].pose.position.x,global_plan[i].pose.position.y);
         }
         printf("start transform plan \n");
       const arc::PoseStamped& plan_pose = global_plan[0];
     //we'll discard points on the plan that are outside the local costmap
//           double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
//                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
           double dist_threshold=1.0;
           printf("dist_threshold=%f\n",dist_threshold);

           unsigned int i = 0;
           double sq_dist_threshold = dist_threshold * dist_threshold;
           double sq_dist = 0;
      //we need to loop to a point on the plan that is within a certain distance of the robot
          while(i < (unsigned int)global_plan.size()) {
              double x_diff =  global_pose.pose.position.x - global_plan[i].pose.position.x;
              double y_diff = global_pose.pose.position.y - global_plan[i].pose.position.y;
              sq_dist = x_diff * x_diff + y_diff * y_diff;
              printf("sq_dist=%f\n",sq_dist);
             if (sq_dist <= sq_dist_threshold) {
                  break;
              }
              ++i;
           }
  //now we'll transform until points are outside of our distance threshold
     while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
      const arc::PoseStamped& pose = global_plan[i];
        transformed_plan.push_back(pose);
        double x_diff = global_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = global_pose.pose.position.y- global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        ++i;
      }
     return true;
 }

  bool getGoalPose(const std::vector<arc::PoseStamped>& global_plan,
      const std::string& global_frame, arc::PoseStamped& goal_pose) {
    if (global_plan.empty())
    {
      printf("Received plan with zero length\n");
      return false;
    }
    else
    {
//        goal_pose.header=goal_pose.header;
//        goal_pose.pose=goal_pose.pose;
        return true;
    }
}

  bool isGoalReached(const std::vector<arc::PoseStamped>& global_plan,
      const costmap_2d_test::Costmap2D_Test& costmap __attribute__((unused)),
      const std::string& global_frame,
      arc::PoseStamped& global_pose,
      const arc::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance){

    //we assume the global goal is the last point in the global plan
    arc::PoseStamped goal_pose;
    getGoalPose(global_plan, global_frame, goal_pose);

    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double goal_th = getYaw(goal_pose.pose.orientation);

    //check to see if we've reached the goal position
    if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      //check to see if the goal orientation has been reached
    if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance) {
        //make sure that we're actually stopped before returning success
       if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
          return true;
      }
    }

    //return false;
  }

  bool stopped(const arc::Odometry& base_odom,
      const double& rot_stopped_velocity, const double& trans_stopped_velocity){
    return fabs(base_odom.twistWithCovariance.twist.angular.z) <= rot_stopped_velocity
      && fabs(base_odom.twistWithCovariance.twist.linear.x) <= trans_stopped_velocity
      && fabs(base_odom.twistWithCovariance.twist.linear.y) <= trans_stopped_velocity;
  }
};
