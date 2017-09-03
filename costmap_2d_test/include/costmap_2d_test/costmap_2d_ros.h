#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

//#include <costmap_2d/layered_costmap.h>
//#include <costmap_2d/layer.h>
//#include <costmap_2d/footprint.h>
//#include <costmap_2d/static_layer.h>
//#include <costmap_2d/obstacle_layer.h>

#include <costmap_2d_test/costmap_2d_test.h>


namespace costmap_2d_test
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS();
  ~Costmap2DROS();

  Costmap2D_Test* getCostmap()
    {
      return &costmap_;
    }
   void getRobotP(double& x, double& y, double& z);
   void start();

protected:
  Costmap2D_Test* costmap2d_;
  Costmap2D_Test costmap_;
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
private:

  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  bool got_footprint_;
  std::vector<arc::Point> unpadded_footprint_;
  std::vector<arc::Point> padded_footprint_;
  float footprint_padding_;
 double robot_x,robot_y,robot_z;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
