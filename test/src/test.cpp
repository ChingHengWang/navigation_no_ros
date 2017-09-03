#include <costmap_2d_test/costmap_2d_test.h>
#include <my_navfn/navfn_no_ros.h>
#include <cv_debug_header.h>
#include <sys/time.h>
#include <math.h>

using namespace std;

double target_x = 0.0,target_y=0.0,target_yaw=0.0;
double current_x = 0.0,current_y=0.0,current_yaw=0.0;
double mobile_delta_x = 0.0;
double mobile_delta_y = 0.0;
double mobile_delta_yaw = 0.0;



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
    FILE *m_s1 = fopen("/home/zach/noros_ws/navigation_no_ros.master/pkg/test/cost.txt", "w");


    arc::PoseStamped startH;
    arc::PoseStamped goalH;
    std::vector<arc::PoseStamped> plan;
    startH.pose.position.x = 0.0;
    startH.pose.position.y = 0.0;
    startH.pose.position.z = 0.0;
    goalH.pose.position.x = 0.0;
    goalH.pose.position.y = 0.0;
    goalH.pose.position.z = 0.0;

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

        if (get_ms() - t0 >= 1000)
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
            imshow("planner_costmap M1", M_1);
            cvWaitKey();
            if(m_s1!=NULL)
            {
                for( int j = 0 ; j < b ; j++ ){
                    for(int i = 0 ; i < a ; i++)
                    {
                        fprintf(m_s1, " %2d ", tmp_map[i + j*b]);
                    }
                    fprintf(m_s1, "\n");
                }
            }
            else
            {
                printf("Can not open timestamp txt file!!!\n");
            } 
            #endif   
            /*plot costmap*/














            /*plot costmap*/
          #if 1
            unsigned int a = costmap2d_test->getSizeInCellsX();
            unsigned int b = costmap2d_test->getSizeInCellsY();
            double res = costmap2d_test->getResolution();
            double origin_x = costmap2d_test->getOriginX();
            double origin_y = costmap2d_test->getOriginY();

            unsigned char* tmp_map;
            tmp_map = costmap2d_test->getCharMap();
            cv::Mat M_1=cv::Mat(b,a,CV_8UC1);
            memcpy(M_1.data,tmp_map,a*b*sizeof(unsigned char));
            flip(M_1,M_1,0);
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
            double sx = startH.pose.position.x;
            double sy = startH.pose.position.y;
            double gx = goalH.pose.position.x;
            double gy = goalH.pose.position.y;
            drawPointWithSize(M_3,sx,sy,0,0,255,0,origin_x,origin_y,res,7);// green
            drawPointWithSize(M_3,gx,gy,0,255,0,0,origin_x,origin_y,res,7);// red
            drawPointVectorWithSize(M_3,current_x,current_y,current_yaw,0,255,0,origin_x,origin_y,res,7,2);// red
 

            imshow("planner_costmap M3", M_3);
            cvWaitKey(100);
            if(m_s1!=NULL)
            {
                for( int j = 0 ; j < b ; j++ ){
                    for(int i = 0 ; i < a ; i++)
                    {
                        fprintf(m_s1, " %2d ", tmp_map[i + j*b]);
                    }
                    fprintf(m_s1, "\n");
                }
            }
            else
            {
                printf("Can not open timestamp txt file!!!\n");
            } 

            
            
            
            //cout<<"origin_x " << origin_x<<" origin_y " << origin_y<<endl;
            //cout<<"current_x " << current_x<<" current_y " << current_y<<" current_yaw "<<current_yaw<<endl;



            #endif   
            /*plot costmap*/

            /* pepper model*/

            current_x = current_x + cos(current_yaw)*mobile_delta_x - sin(current_yaw)*mobile_delta_y;
            current_y = current_y + sin(current_yaw)*mobile_delta_x + cos(current_yaw)*mobile_delta_y;
            current_yaw = current_yaw + mobile_delta_yaw;





        }


        if (get_ms() - t1 >= 100)
        {
            //cout << "duration time: " << get_ms()-t1 << endl;
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
            int buf_index = 8;
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

            //cout << "target x " << target_x << " target y" << target_y<<endl;
            //cout << "target yaw" << target_yaw * double(360/6.28)<<endl;

            /*calculate diff pos and yaw*/
            double delta_x = target_x - current_x; 
            double delta_y = target_y - current_y; 
            
            double delta_a = target_yaw - current_yaw - a;
            double angle_step = 1.0;
            delta_a = (delta_a >= angle_step) ? angle_step : ((delta_a <= -angle_step) ? -angle_step : delta_a);
            a = a +delta_a;
            double delta_yaw = a ;
        

            


             /* limit*/
            double trans_step = 1.0;
            delta_x = (delta_x >= trans_step) ? trans_step : ((delta_x <= -trans_step) ? -trans_step : delta_x);
            delta_y = (delta_y >= trans_step) ? trans_step : ((delta_y <= -trans_step) ? -trans_step : delta_y);


            
            /*transfer to mobile coordinate*/
            mobile_delta_x = cos(current_yaw)*delta_x + sin(current_yaw)*delta_y;
            mobile_delta_y = -sin(current_yaw)*delta_x + cos(current_yaw)*delta_y;
            mobile_delta_yaw = delta_yaw;

            //std::cout << "x:"<<mobile_delta_x<<" y:"<<mobile_delta_y<<" th:"<<mobile_delta_yaw<<std::endl;
            printf("dx:%10f dy:%10f dw:%10f mv_x:%10f mv_y:%10f mv_yaw:%10f\n",delta_x,delta_y,delta_yaw,mobile_delta_x,mobile_delta_y,mobile_delta_yaw);



   
            /* pepper model*/
/*
            current_x = current_x + cos(current_yaw)*mobile_delta_x - sin(current_yaw)*mobile_delta_y;
            current_y = current_y + sin(current_yaw)*mobile_delta_x + cos(current_yaw)*mobile_delta_y;
            current_yaw = current_yaw + mobile_delta_yaw;

*/


        /* cmd_vel = calcualte_speed() */

        /* send to pepper */
        }
    }
 
  


}
