# my_navfn

* 宣告
		costmap_2d::Costmap2DROS* planner_costmap_ros_;

                navfn::NavfnNoROS* test_planner_;


* 初始化時先給NULL

		planner_costmap_ros_(NULL)

* 實體化
		planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);

		planner_costmap_ros_->pause();

		planner_costmap_ros_->start();


 		test_planner_ = new navfn::NavfnNoROS();

		test_planner_->initializeNoRos(planner_costmap_ros_->getCostmap());


* startH : Header::PoseStamped 
* goalH : Header::PoseStamped 

		test_planner_->makePlanNoRos(startH,goalH,0.0,*test_plan_);




		planner_costmap_ros_->stop();

