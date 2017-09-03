#include <my_dwa_local_planner/my_dwa_planner_wrapper.h>
#include <Eigen/Core>
#include <cmath>



#include <base_local_planner_test/goal_functions.h>

//#include <debug/cv_debug_header.h>
//register this planner as a BaseLocalPlanner plugin
//PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerWrapper, nav_core::BaseLocalPlanner)

namespace my_dwa_local_planner {





DWAPlannerWrapper::DWAPlannerWrapper()
{

}
  void DWAPlannerWrapper::initialize(
      std::string name,

      costmap_2d_test::Costmap2D_Test* costmap_ros) {


      costmap_ros_ = costmap_ros;

      costmap_ros_->getRobotPose(tmp_x,tmp_y,tmp_yaw);
	  current_pose_.pose.position.x = tmp_x;
	  current_pose_.pose.position.y = tmp_y;
	  tmp_euler.x=0;tmp_euler.y=0;tmp_euler.z=tmp_yaw;
	  current_pose_.pose.orientation = getQuaternion(tmp_euler);

	  
      // make sure to update the costmap we'll use for this cycle
      costmap_2d_test::Costmap2D_Test* costmap = costmap_ros_;

      //planner_util_.initialize(costmap, costmap_ros_->getGlobalFrameID());
	  planner_util_.initialize(costmap, "odom");
      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));
  	  odom_helper_.setOdomTopic( "odom" );

      


 

  }
  
  bool DWAPlannerWrapper::setPlan(const std::vector<arc::PoseStamped>& orig_global_plan) {
  //fix : orig_global_plan keep PoseStamped
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    printf("Got new plan\n");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerWrapper::isGoalReached() {

    if ( ! costmap_ros_->getRobotPose(tmp_x,tmp_y,tmp_yaw)) {
      printf("Could not get robot pose");
      return false;
    }
	  current_pose_.pose.position.x = tmp_x;
	  current_pose_.pose.position.y = tmp_y;
	  tmp_euler.x = 0;tmp_euler.y = 0;tmp_euler.z = tmp_yaw;
	  current_pose_.pose.orientation = getQuaternion(tmp_euler);

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      printf("Goal reached");
      return true;
    } else {
      return false;
    }
  }




  DWAPlannerWrapper::~DWAPlannerWrapper(){
    //make sure to clean things up

  }


  bool DWAPlannerWrapper::dwaComputeVelocityCommands(arc::PoseStamped &global_pose, arc::Twist& cmd_vel) {
    printf("[dwa] Start DWAcomputeVelocityCommands \n");


    // dynamic window sampling approach to get useful velocity commands


    arc::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel); //odom velocity


    //compute what trajectory to drive along
    arc::PoseStamped drive_cmds;
    //drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    drive_cmds.header.frame_id = "base_foot_print";
    // call with updated footprint
    base_local_planner_test::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());
    printf("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

  #if 0
/////////////////////////////////////////////// ZACH DEBUG
    printf("[dwa] start plot dwa costmap\n");
    costmap_2d::Costmap2D* tmp_costmap_ = costmap_ros_->getCostmap();

    unsigned int a = tmp_costmap_->getSizeInCellsX();
    unsigned int b = tmp_costmap_->getSizeInCellsY();
    double res = tmp_costmap_->getResolution();
    double origin_x = tmp_costmap_->getOriginX();
    double origin_y = tmp_costmap_->getOriginY();
    unsigned char* tmp_map;
    tmp_map = tmp_costmap_->getCharMap();
    cv::Mat M_1=cv::Mat(b,a,CV_8UC1);
    memcpy(M_1.data,tmp_map,a*b*sizeof(unsigned char));
    //transpose(M_1,M_1);
    flip(M_1,M_1,0);
    cv::Mat M_1_b;
    createOpenCVDebugMatForCostmap(M_1, M_1_b);
    cv::Mat M_3;
    cv::cvtColor( M_1_b, M_3, CV_GRAY2RGB);
    int i=0;
    unsigned int size_of_plan = path.getPointsSize();
    double plan_x = 0, plan_y = 0,plan_th=0.0;
    while(i<size_of_plan){
      path.getPoint(i,plan_x,plan_y,plan_th);
      //printf("ZACH DEBUG : size is %d path is %f %f\n",size_of_plan,plan_x,plan_y);
      i++;
      //drawPose(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res);
      drawPointWithSize(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res,2);
    }
    imshow("dwa_planner_costmap ", M_3);
    cvWaitKey(10);

/////////////////////////////////////////////// ZACH DEBUG END
#endif















    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<arc::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      printf("dwa_local_planner : The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();

      return false;
    }

    printf("dwa_local_planner : A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      arc::PoseStamped p;
	  //p.header.frame_id = costmap_ros_->getGlobalFrameID();
	  p.header.frame_id = "odom";
	  p.pose.position.x = p_x;
	  p.pose.position.y = p_y;
	  p.pose.position.z = 0;
	  
	  tmp_euler.x = 0;tmp_euler.y = 0; tmp_euler.z = p_th;
      p.pose.orientation = getQuaternion(tmp_euler);

                    
    
                      
         
      arc::PoseStamped pose;

      local_plan.push_back(pose);
    }

    return true;
  }




  bool DWAPlannerWrapper::computeVelocityCommands(arc::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    printf("[dwa] Start computeVelocityCommands \n");

    if ( ! costmap_ros_->getRobotPose(tmp_x,tmp_y,tmp_yaw)) {
      printf("Could not get robot pose");
      return false;
    }
	  current_pose_.pose.position.x = tmp_x;
	  current_pose_.pose.position.y = tmp_y;
	  tmp_euler.x=0;tmp_euler.y=0;tmp_euler.z=tmp_yaw;
	  current_pose_.pose.orientation = getQuaternion(tmp_euler);	

    std::vector<arc::PoseStamped> transformed_plan;
  //  transformed_plan = planner_util_.getLocalPlan(current_pose_);
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      printf("Could not get local plan");
      return false;
    }
    int m=transformed_plan.size();
    printf("msss=%d\n",m);
    for(int i=0; i<m;i++)
    {
    	printf("point: %p, pp: %p, locax=%f localy=%f\n",&transformed_plan, &(transformed_plan[i].pose.position.x), transformed_plan[i].pose.position.x,transformed_plan[i].pose.position.y);
    }
    printf("o\n");
    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      printf("dwa_local_planner : Received an empty transformed plan.\n");
      return false;
    }
    printf("u\n");
    printf("dwa_local_planner : Received a transformed plan with %zu points.", transformed_plan.size());
    int n=transformed_plan.size();
    printf("nsss=%d\n",n);
    for(int i=0; i<n;i++)
    {
    	printf("locax1=%f localy1=%f\n",transformed_plan[i].pose.position.x,transformed_plan[i].pose.position.y);
    }
    printf("l\n");
    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan); //set target to cost function
    printf("d1\n");
    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<arc::PoseStamped> local_plan;
      std::vector<arc::PoseStamped> transformed_plan;

      base_local_planner_test::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      printf("[dwa] prepare to dwaComputeVelocity\n");
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        //publishGlobalPlan(transformed_plan);
      } else {
        printf("dwa_local_planner : DWA planner failed to produce path.");
        std::vector<arc::PoseStamped> empty_plan;

      }
      return isOk;
    }
  }


};
