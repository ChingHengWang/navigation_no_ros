
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <costmap_2d_test/header.h>


#include <angles/angles.h>



#include <costmap_2d_test/costmap_2d_test.h>
#include <base_local_planner_test/latched_stop_rotate_controller.h>

#include <base_local_planner_test/odometry_helper_ros.h>

#include <my_dwa_local_planner/my_dwa_planner.h>

namespace my_dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerWrapper {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerWrapper();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name,
          costmap_2d_test::Costmap2D_Test* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerWrapper();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(arc::Twist& cmd_vel);


      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(arc::PoseStamped& global_pose, arc::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<arc::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();




    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */



      base_local_planner_test::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d_test::Costmap2D_Test* costmap_ros_;


      bool setup_;
      arc::PoseStamped current_pose_;

      base_local_planner_test::LatchedStopRotateController latchedStopRotateController_;






      base_local_planner_test::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
	  
	  double tmp_x ,tmp_y ,tmp_yaw; // workaround for getRobotPose
	  arc::Vector3 tmp_euler;
  };
};
#endif
