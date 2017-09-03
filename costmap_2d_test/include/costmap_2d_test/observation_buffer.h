#ifndef COSTMAP_2D_OBSERVATION_BUFFER_H_
#define COSTMAP_2D_OBSERVATION_BUFFER_H_

#include <vector>
#include <list>
#include <string>
#include <costmap_2d_test/observation.h>
#include <boost/thread.hpp>
//#include <costmap_2d/header.h>

namespace costmap_2d_test
{
/**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
class ObservationBuffer
{
public:
  ObservationBuffer(double observation_keep_time, double expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, std::string global_frame,
                    std::string sensor_frame);

  /**
   * @brief  Destructor... cleans up
   */
  ~ObservationBuffer();

  /**
   * @brief Sets the global frame of an observation buffer. This will
   * transform all the currently cached observations to the new global
   * frame
   * @param new_global_frame The name of the new global frame.
   * @return True if the operation succeeds, false otherwise
   */
  bool setGlobalFrame(const std::string new_global_frame);

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
   * @param  cloud The cloud to be buffered
   */
  void bufferCloud( const arc::PointCloud& cloud);

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
   * @param  cloud The cloud to be buffered
   */
  void bufferCloud(const pointcloud& cloud);

  /**
   * @brief  Pushes copies of all current observations onto the end of the vector passed in
   * @param  observations The vector to be filled
   */
  void getObservations(std::vector<Observation>& observations);

  /**
   * @brief  Check if the observation buffer is being update at its expected rate
   * @return True if it is being updated at the expected rate, false otherwise
   */
  bool isCurrent() const;

  /**
   * @brief  Lock the observation buffer
   */
  inline void lock()
  {
    lock_.lock();
  }

  /**
   * @brief  Lock the observation buffer
   */
  inline void unlock()
  {
    lock_.unlock();
  }

  /**
   * @brief Reset last updated timestamp
   */
  void resetLastUpdated();

  //void transformPointCloud(std::string new_global_frame, const pointcloud& pc_in,  pointcloud& pc_out,   float R[9],   float T[3]);
 // void  transformPoint( std::string new_global_frame, const arc::PointStamped origin_in, arc::PointStamped origin_out,  float R[9],  float T[3]);

private:
  /**
   * @brief  Removes any stale observations from the buffer list
   */
  void purgeStaleObservations();

  std::string global_frame_;
  std::string sensor_frame_;
  float R[9],T[3];
  std::list<Observation> observation_list_;
  std::string topic_name_;
  long last_updated_;
  long observation_keep_time_;
  long expected_update_rate_;
  double min_obstacle_height_, max_obstacle_height_;
  boost::recursive_mutex lock_;  ///< @brief A lock for accessing data in callbacks safely
  double obstacle_range_, raytrace_range_;
  double tf_tolerance_;
};
}  // namespace costmap_2d
#endif  // COSTMAP_2D_OBSERVATION_BUFFER_H_
