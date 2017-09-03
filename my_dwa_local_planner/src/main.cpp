#include <costmap_2d_test/costmap_2d_test.h>
#include <my_navfn/navfn_no_ros.h>
#include <my_dwa_local_planner/my_dwa_planner_wrapper.h>

int main(int argc, char **argv)
{
   FILE* fp=fopen("/home/lixiaopeng/pepper_workspace/src/pepper_nav/data/path.dat","w");
   costmap_2d_test::Costmap2D_Test* costmap2d_test;
   costmap2d_test=new costmap_2d_test::Costmap2D_Test();
   printf("HELLO PEPPER\n");
   costmap2d_test->updateMap();

   navfn::NavfnNoROS* test_planner_;
   test_planner_ = new navfn::NavfnNoROS();
   test_planner_->initializeNoRos(costmap2d_test);
   
   arc::PoseStamped startH;
   arc::PoseStamped goalH;
   std::vector<arc::PoseStamped> plan;

   startH.pose.position.x = 0.0;
   startH.pose.position.y = 0.0;
   startH.pose.position.z = 0.0;
   unsigned int mx;
   unsigned int my;
   costmap2d_test->worldToMap(startH.pose.position.x, startH.pose.position.y, mx, my);
   printf("mx=%d my=%d\n",mx,my);
   unsigned char cost_value;
   cost_value = costmap2d_test->getCost(mx, my);
   printf("cost_value=%d\n",cost_value);
   goalH.pose.position.x = 0.0;
   goalH.pose.position.y = 2.0;
   goalH.pose.position.z = 0.0;
   unsigned int mx1;
   unsigned int my1;
   costmap2d_test->worldToMap(goalH.pose.position.x, goalH.pose.position.y, mx1, my1);
   printf("mx1=%d my1=%d\n",mx1,my1);
   unsigned char cost_value1;
   cost_value1 = costmap2d_test->getCost(mx1, my1);
   printf("cost_value1=%d\n",cost_value1);
    unsigned int X;
    unsigned int Y;
    X=costmap2d_test->getSizeInCellsX();
    Y=costmap2d_test->getSizeInCellsY();
   printf("X=%d Y=%d\n",X,Y);
   double X0,Y0;
   X0=costmap2d_test->getSizeInMetersX();
   Y0=costmap2d_test->getSizeInMetersY();
   printf("X0=%f Y0=%f\n",X0,Y0);
   double originx,originy;
   originx=costmap2d_test->getOriginX();
   originy=costmap2d_test->getOriginY();
   printf("originx=%f originy=%f\n",originx,originy);
   
   unsigned char cost_value0 = costmap2d_test->getCost(323, 355);
   printf("cost_value0=%d\n",cost_value0);
   unsigned int mxo,myo;
   costmap2d_test->worldToMap(-7.0, -8.8, mxo, myo);
   printf("mxo=%d myo=%d\n",mxo,myo);
   unsigned char cost_value00 = costmap2d_test->getCost(mxo, myo);
   printf("cost_value00=%d\n",cost_value00);
   test_planner_->makePlanNoRos(startH,goalH,0.0,plan);
   
   int size_total = plan.size();
   printf("size_total=%d\n",size_total);
   for (int i = 0; i <size_total; ++i)
   {
       fprintf(fp,"%f %f\n",plan[i].pose.position.x,plan[i].pose.position.y);
   }  
   
   my_dwa_local_planner::DWAPlannerWrapper* dw;
   dw=new my_dwa_local_planner::DWAPlannerWrapper();
   printf("h\n");
   std::string dwa;
   dw->initialize(dwa, costmap2d_test);
   printf("e\n");
   dw->setPlan(plan);
    printf("l\n");
   arc::Twist cmd_vel;
   dw->computeVelocityCommands( cmd_vel);

}
