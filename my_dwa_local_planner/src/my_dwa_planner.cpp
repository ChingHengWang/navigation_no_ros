#include <my_dwa_local_planner/my_dwa_planner.h>
#include <base_local_planner_test/goal_functions.h>
#include <base_local_planner_test/map_grid_cost_point.h>
#include <cmath>

#include <queue>

#include <angles/angles.h>
#include <costmap_2d_test/header.h>


namespace my_dwa_local_planner {

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












  DWAPlanner::DWAPlanner(std::string name, base_local_planner_test::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    

    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );


      double controller_frequency = 5.0;

      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency; //0.2
      } else {
        printf("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    
    printf("Sim period is set to %.2f", sim_period_);
    printf("initialize dwaplanner\n");

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores = false;
 
    obstacle_costs_.setSumScores(sum_scores);



    std::string frame_id = "odom";
 


    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner_test::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation

    // trajectory generators
    std::vector<base_local_planner_test::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner_test::SimpleScoredSamplingPlanner(generator_list, critics);

	cheat_factor_ = 1.0;
  }

  // used for visualization only, total_costs are not really total costs
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d_test::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double resolution = planner_util_->getCostmap()->getResolution();
    total_cost =
        pdist_scale_ * resolution * path_cost +
        gdist_scale_ * resolution * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool DWAPlanner::setPlan(const std::vector<arc::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    printf("enter setplan\n");
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool DWAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner_test::Trajectory traj;
    arc::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, getYaw(goal_pose.pose.orientation));
    base_local_planner_test::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    printf("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }


  void DWAPlanner::updatePlanAndLocalCosts(
      arc::PoseStamped global_pose,//odom
                                        // from controller_costmap
      const std::vector<arc::PoseStamped>& new_plan) {
	  printf("enter updateplan\n");
    global_plan_.resize(new_plan.size());

    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i]; // set transform_plan to dwa_planer's global_plan
      printf("gx=%f gy=%f\n",global_plan_[i].pose.position.x,global_plan_[i].pose.position.y);
    }

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);
    //MapGridCostFunction 
    //

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    // alignment costs
    arc::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<arc::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    
    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      double resolution = planner_util_->getCostmap()->getResolution();
      alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
    }
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner_test::Trajectory DWAPlanner::findBestPath(
      arc::PoseStamped global_pose,
      arc::PoseStamped global_vel,
      arc::PoseStamped& drive_velocities,
      std::vector<arc::Point> footprint_spec) {

    obstacle_costs_.setFootprint(footprint_spec);


    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.y, global_vel.pose.position.y, getYaw(global_vel.pose.orientation));
    arc::PoseStamped goal_pose = global_plan_.back();

    printf("dwa_planner_debug : global_plan_frame_id %s\n",global_plan_.back().header.frame_id.c_str());
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, getYaw(goal_pose.pose.orientation));
    base_local_planner_test::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    printf("dwa_planner_debug : get limit %s\n",global_plan_.back().header.frame_id.c_str());
 
    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,//from locl_planner_util.limit
        vsamples_);
    printf("dwa_planner_debug : vsamples_[0 1 2] %f %f %f\n",vsamples_[0],vsamples_[1],vsamples_[2]); 
    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner_test::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
    //result_traj is the best traj!!
    //all_expored is all explored trajs


    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose.position.x = 0;
	  drive_velocities.pose.position.y = 0;
	  drive_velocities.pose.position.z = 0;
	  drive_velocities.pose.orientation.x = 0;
	  drive_velocities.pose.orientation.y = 0;
	  drive_velocities.pose.orientation.z = 0;
	  drive_velocities.pose.orientation.w = 1;
    } else {
      drive_velocities.pose.position.x = result_traj_.xv_;
	  drive_velocities.pose.position.y = result_traj_.yv_;
	  drive_velocities.pose.position.z = 0;	 
	  
	  tmp_euler.x = 0;
	  tmp_euler.y = 0; 
	  tmp_euler.z = result_traj_.thetav_;

      drive_velocities.pose.orientation = getQuaternion(tmp_euler);
	  //free(&tmp_euler);

    }

    return result_traj_;
  }
};
