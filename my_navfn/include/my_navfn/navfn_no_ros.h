#ifndef MY_NAVFN_NAVFN_NO_ROS_H_
#define MY_NAVFN_NAVFN_NO_ROS_H_

#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d_test.h>
#include <vector>
namespace navfn {
  class NavfnNoROS {
    public:
      NavfnNoROS();

      NavfnNoROS(costmap_2d_test::Costmap2D_Test* costmap);

      void initializeNoRos(costmap_2d_test::Costmap2D_Test* costmap);

      bool makePlanNoRos(const arc::PoseStamped& start,const arc::PoseStamped& goal,double tolerance, std::vector<arc::PoseStamped>& plan);

      bool getPlanFromPotentialNoRos(const arc::PoseStamped& goal, std::vector<arc::PoseStamped>& plan);

      ~NavfnNoROS(){}

    protected:

      costmap_2d_test::Costmap2D_Test* costmap_;
      boost::shared_ptr<NavFn> planner_;
      bool allow_unknown_;


    private:

      void mapToWorld(double mx, double my, double& wx, double& wy);

  };
};

#endif
