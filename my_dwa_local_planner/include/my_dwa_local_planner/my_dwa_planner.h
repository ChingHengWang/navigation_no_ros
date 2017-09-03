
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>




//for creating a local cost grid



//for obstacle data access
#include <costmap_2d_test/costmap_2d_test.h>

#include <base_local_planner_test/trajectory.h>
#include <base_local_planner_test/local_planner_limits.h>
#include <base_local_planner_test/local_planner_util.h>
#include <base_local_planner_test/simple_trajectory_generator.h>

#include <base_local_planner_test/oscillation_cost_function.h>
#include <base_local_planner_test/map_grid_cost_function.h>
#include <base_local_planner_test/obstacle_cost_function.h>
#include <base_local_planner_test/simple_scored_sampling_planner.h>

#include <costmap_2d_test/header.h>
namespace my_dwa_local_planner {

  	  double getYaw(const arc::Quaternion orientation);
  	  arc::Quaternion getQuaternion(const arc::Vector3 euler );	  
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      DWAPlanner(std::string name, base_local_planner_test::LocalPlannerUtil *planner_util);

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlanner() {}



      /**
       * @brief  Check if a trajectory is legal for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner_test::Trajectory findBestPath(
          arc::PoseStamped global_pose,
          arc::PoseStamped global_vel,
          arc::PoseStamped& drive_velocities,
          std::vector<arc::Point> footprint_spec);

      /**
       * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
       * @param  new_plan The new global plan
       */
      void updatePlanAndLocalCosts(arc::PoseStamped global_pose,
          const std::vector<arc::PoseStamped>& new_plan);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * sets new plan and resets state
       */
      bool setPlan(const std::vector<arc::PoseStamped>& orig_global_plan);

    private:

      base_local_planner_test::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double pdist_scale_, gdist_scale_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      base_local_planner_test::Trajectory result_traj_;

      double forward_point_distance_;

      std::vector<arc::PoseStamped> global_plan_;



      double cheat_factor_;
	  arc::Vector3 tmp_euler; //for debug

      // see constructor body for explanations
      base_local_planner_test::SimpleTrajectoryGenerator generator_;
      base_local_planner_test::OscillationCostFunction oscillation_costs_;
      base_local_planner_test::ObstacleCostFunction obstacle_costs_;
      base_local_planner_test::MapGridCostFunction path_costs_;
      base_local_planner_test::MapGridCostFunction goal_costs_;
      base_local_planner_test::MapGridCostFunction goal_front_costs_;
      base_local_planner_test::MapGridCostFunction alignment_costs_;

      base_local_planner_test::SimpleScoredSamplingPlanner scored_sampling_planner_;


  };
};
#endif
