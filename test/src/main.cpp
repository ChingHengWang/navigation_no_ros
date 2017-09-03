#include <costmap_2d_test/costmap_2d_test.h>
#include <my_navfn/navfn_no_ros.h>
#include <cv_debug_header.h>
#include <sys/time.h>
#include <math.h>

using namespace std;


double t0=0.0,t1=0.0;
double get_ms()
{
  struct timeval t0;
  gettimeofday(&t0,NULL);
  double ret = t0.tv_sec * 1000.0;
  ret += ((double)t0.tv_usec)*0.001;
  return ret;
}
   


int main(int argc, char **argv)
{
/* initial variable */
 
    arc::PoseStamped startH;
    arc::PoseStamped goalH;
    std::vector<arc::PoseStamped> plan;
    startH.pose.position.x = 0.0;
    startH.pose.position.y = 0.0;
    startH.pose.position.z = 0.0;
    goalH.pose.position.x = 0.0;
    goalH.pose.position.y = 0.0;
    goalH.pose.position.z = 0.0;
    double target_x = 0.0,target_y=0.0,target_yaw=0.0;
    double current_x = 0.0,current_y=0.0,current_yaw=0.0;
    // diff angle
    double a = 0.0;
    // move step for trans and angle
    double trans_step = 0.02;
    double angle_step = 0.2;

   
    
/* initial costmap */
    costmap_2d_test::Costmap2D_Test* costmap2d_test;
    costmap2d_test=new costmap_2d_test::Costmap2D_Test();
    costmap2d_test->updateMap();

    /* initial global planner */
    navfn::NavfnNoROS* test_planner_;
    test_planner_ = new navfn::NavfnNoROS();
    test_planner_->initializeNoRos(costmap2d_test);


/*while loop*/
    while(1)
    {  

        if (get_ms() - t0 >= 500)
        {
            //cout << "map time: " << get_ms()-t0 << endl;
            t0 = get_ms();

            /*plot costmap*/
          #if 0
            unsigned int a = costmap2d_test->getSizeInCellsX();
            unsigned int b = costmap2d_test->getSizeInCellsY();
            double res = costmap2d_test->getResolution();
            double origin_x = costmap2d_test->getOriginX();
            double origin_y = costmap2d_test->getOriginY();

            unsigned char* tmp_map;
            tmp_map = costmap2d_test->getCharMap();
            cv::Mat M_1=cv::Mat(b,a,CV_8UC1);
            memcpy(M_1.data,tmp_map,a*b*sizeof(unsigned char));
            cv::Mat M_1_b;
            createOpenCVDebugMatForCostmap(M_1, M_1_b);
            cv::Mat M_3;
            cv::cvtColor( M_1_b, M_3, CV_GRAY2RGB);

            int i = 0;
            std::vector<arc::PoseStamped> tmp_plan_ = plan;
            unsigned int size_of_plan = plan.size();
            double plan_x = 0, plan_y = 0;
            while(i<size_of_plan){
              plan_x = plan[i].pose.position.x;
              plan_y = plan[i].pose.position.y;
              i++;
              drawPointWithSize(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res,2);
            }

            drawPointWithSize(M_3,0.0,0.0,0,255,0,0,origin_x,origin_y,res,2);
            drawPointWithSize(M_3,1.0,0.0,0,0,255,0,origin_x,origin_y,res,2);
            drawPointWithSize(M_3,0.0,1.0,0,0,0,255,origin_x,origin_y,res,2);


            imshow("planner_costmap M1", M_1);
            cvWaitKey(100);
            cout<<"origin_x " << origin_x<<endl;
            cout<<"origin_y " << origin_y<<endl;


#endif   
            /*plot costmap*/

        }


        if (get_ms() - t1 >= 100)
        {
            cout << "duration time: " << get_ms()-t1 << endl;
            t1 = get_ms();

            
        /*  cmd from web*/
            goalH.pose.position.x = 4.0;
            goalH.pose.position.y = 12.0;
             
        /*  get position from sensor */

            startH.pose.position.x = current_x ;
            startH.pose.position.y = current_y ;
 

        /* path = global_path(start ,end) */
            test_planner_->makePlanNoRos(startH,goalH,0.0,plan);


            /*calculation target position and yaw*/
            int buf_index = 10;
            target_x = plan[buf_index].pose.position.x;
            target_y = plan[buf_index].pose.position.y;
            double target_delta_x = plan[buf_index+1].pose.position.x - plan[buf_index-1].pose.position.x;
            double target_delta_y = plan[buf_index+1].pose.position.y - plan[buf_index-1].pose.position.y;
            target_delta_x = (target_delta_x == 0.0) ? 0.001 : target_delta_x;

            // 1 cordinate
            if ( target_delta_x > 0 && target_delta_y >= 0 ) 
                target_yaw = atan(target_delta_y / target_delta_x);
            // 2 coordinate
            else if ( target_delta_x < 0 && target_delta_y >= 0 ) 
                 target_yaw = M_PI + atan(target_delta_y / target_delta_x);
            // 3 coordinate
            else if ( target_delta_x < 0 && target_delta_y < 0 ) 
                 target_yaw = -M_PI + atan(target_delta_y / target_delta_x);
            // 4 coordinate
            else if ( target_delta_x > 0 && target_delta_y < 0 ) 
                 target_yaw = atan(target_delta_y / target_delta_x);

            cout << "target x" << target_x<<endl;
            cout << "target y" << target_y<<endl;
            cout << "target yaw" << target_yaw * double(360/6.28)<<endl;

            /*calculate diff pos and yaw*/
            double delta_x = target_x - current_x; 
            double delta_y = target_y - current_y; 
            double trans_step = 0.02;
            double angle_step = 0.2;
            delta_x = (delta_x >= trans_step) ? trans_step : ((delta_x <= -trans_step) ? -trans_step : delta_x);
            delta_y = (delta_y >= trans_step) ? trans_step : ((delta_y <= -trans_step) ? -trans_step : delta_y);


            double delta_a = target_yaw - current_yaw - a;
            delta_a = (delta_a >= angle_step) ? angle_step : ((delta_a <= -angle_step) ? -angle_step : delta_a);
            a = a +delta_a;
            double delta_yaw = a ;
        

            /* pepper model*/
            current_x = current_x + delta_x;
            current_y = current_y + delta_y;
            current_yaw = current_yaw + delta_yaw;
        


        /* cmd_vel = calcualte_speed() */

        /* send to pepper */
        }
    }
 
  


}
