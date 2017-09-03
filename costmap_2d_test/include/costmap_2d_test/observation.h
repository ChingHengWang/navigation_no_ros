#ifndef COSTMAP_2D_OBSERVATION_H_
#define COSTMAP_2D_OBSERVATION_H_

#include <costmap_2d_test/pointcloud.h>

namespace costmap_2d_test
{

class Observation
{
public:
  /**
   * @brief  Creates an empty observation
   */
  Observation() :
   cloud_(new pointcloud()), obstacle_range_(0.0), raytrace_range_(0.0)
  {
  }

  virtual ~Observation()
  {
    delete cloud_;
  }

  Observation(arc::Point& origin, const pointcloud cloud,
              double obstacle_range, double raytrace_range) :
      origin_(origin), cloud_(new pointcloud(cloud)),
      obstacle_range_(obstacle_range), raytrace_range_(raytrace_range)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation& obs) :
      origin_(obs.origin_), cloud_(new pointcloud(*(obs.cloud_))),
      obstacle_range_(obs.obstacle_range_), raytrace_range_(obs.raytrace_range_)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param cloud The point cloud of the observation
   * @param obstacle_range The range out to which an observation should be able to insert obstacles
   */
  Observation( const pointcloud cloud, double obstacle_range) :
      cloud_(new pointcloud(cloud)), obstacle_range_(obstacle_range), raytrace_range_(0.0)
  {
  }

  arc::Point origin_;
  pointcloud* cloud_;
  double obstacle_range_, raytrace_range_;

};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_OBSERVATION_H_
