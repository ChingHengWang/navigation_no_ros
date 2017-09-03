#include <costmap_2d_test/observation_buffer.h>
using namespace std;

namespace costmap_2d_test
{
ObservationBuffer::ObservationBuffer(double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, string global_frame,
                                     string sensor_frame) :
    observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    global_frame_(global_frame), sensor_frame_(sensor_frame),min_obstacle_height_(min_obstacle_height),
     max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range)
{
	 R[9]= (1.0, 0.0, 0.0, 0.0, 1.0 ,0.0 , 0.0, 0.0 ,1.0);
     T[3]= (0.0, 0.0 ,1.0);
}

ObservationBuffer::~ObservationBuffer()
{
}

//将观察缓冲器转化到新的坐标系
bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
    struct timeval t_start;
    gettimeofday(&t_start, NULL);
   long transform_time = ((long)t_start.tv_sec)*1000+(long)t_start.tv_usec/1000;
	arc::PointStamped origin_in;
	arc::PointStamped origin_out;
    list<Observation>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
        Observation& obs = *obs_it;
        origin_in.header.frame_id = global_frame_;
        origin_in.header.stamp = transform_time;
        origin_in.point = obs.origin_;
     //   transformPoint(new_global_frame, origin_in, origin_out, R[9], T[3]);
        obs.origin_ = origin_out.point;
      // transformPointCloud(new_global_frame, *obs.cloud_, *obs.cloud_, R[9], T[3]);
        obs.cloud_->header.frame_id =new_global_frame;
    }

    // now we need to update our global_frame member
    global_frame_ = new_global_frame;
    return true;
}
//
void ObservationBuffer::bufferCloud( const arc::PointCloud& cloud)
{

    pointcloud cloud_tem;
    cloud_tem.header.frame_id=cloud.header.frame_id;
    cloud_tem.header.stamp=cloud.header.stamp;
    cloud_tem.Point=cloud;
    bufferCloud(cloud_tem);

}

void ObservationBuffer::bufferCloud(const pointcloud& cloud)
{
    arc::PointStamped global_origin;
    // create a new observation on the list to be populated
    observation_list_.push_front(Observation());
    // check whether the origin frame has been set explicitly or whether we should get it from the cloud
    string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    arc::PointStamped local_origin;
    local_origin.header.frame_id=origin_frame;
    local_origin.point.x=0.0;
    local_origin.point.y=0.0;
    local_origin.point.z=0.0;
   // transformPoint(global_frame_, local_origin, global_origin, R[9], T[3]);
    observation_list_.front().origin_.x = global_origin.point.x;
    observation_list_.front().origin_.y = global_origin.point.y;
    observation_list_.front().origin_.z = global_origin.point.z;

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    pointcloud global_frame_cloud;

    // transform the point cloud
  // transformPointCloud(global_frame_, cloud, global_frame_cloud, R[9], T[3]);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    pointcloud &observation_cloud = *(observation_list_.front().cloud_);
    unsigned int cloud_size = global_frame_cloud.Point.points.size();
    observation_cloud.Point.points.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
        if (global_frame_cloud.Point.points[i].z <= max_obstacle_height_
                && global_frame_cloud.Point.points[i].z >= min_obstacle_height_)
        {
            observation_cloud.Point.points[point_count++] = global_frame_cloud.Point.points[i];
        }
    }

    // resize the cloud for the number of legal points
    observation_cloud.Point.points.resize(point_count);
     observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    // if the update was successful, we want to update the last updated time
    struct timeval t_start;
    gettimeofday(&t_start, NULL);
    last_updated_ = ((long)t_start.tv_sec)*1000+(long)t_start.tv_usec/1000;
    // we'll also remove any stale observations from the list
    purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
    // first... let's make sure that we don't have any stale observations
    purgeStaleObservations();

    // now we'll just copy the observations for the caller
    list<Observation>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
        observations.push_back(*obs_it);
    }
}

void ObservationBuffer::purgeStaleObservations()
{

    if (!observation_list_.empty())
    {
        list<Observation>::iterator obs_it = observation_list_.begin();
        // if we're keeping observations for no time... then we'll only keep one observation

        if (observation_keep_time_ == 0.0)
        {
            observation_list_.erase(++obs_it, observation_list_.end());
            return;
        }
    }
}

bool ObservationBuffer::isCurrent() const
{
   return true;
}

void ObservationBuffer::resetLastUpdated()
{
   struct timeval t_start;
   gettimeofday(&t_start, NULL);
   last_updated_ = ((long)t_start.tv_sec)*1000+(long)t_start.tv_usec/1000;
}
/*
void ObservationBuffer::transformPointCloud(std::string new_global_frame, const pointcloud& pc_in,  pointcloud& pc_out,    float R[9],   float T[3])
{
    if ( pc_in.header.frame_id == new_global_frame)
    {
        pc_out=pc_in;
    }
	R[9]=  (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ,1.0);
	T[3]= (0.0, 0.0 ,1.0);
	const int num = pc_in.Point.points.size();
    pc_out.Point.points.resize( num );
    pc_out.header=pc_in.header;
    arc::Point point;
    for(int i=0; i<num; i++ )
    {
        float x=pc_in.Point.points[i].x;
        float y=pc_in.Point.points[i].y;
        float z=pc_in.Point.points[i].z;

        point.x = R[0] * x + R[1] * y + R[2] * z + T[0];
        point.y = R[3] * x + R[4] * y + R[5] * z + T[1];
        point.z = R[6] * x + R[7] * y + R[8] * z + T[2];
        pc_out.Point.points.push_back(point);
    }
    pc_out.header.frame_id=new_global_frame;
}

void ObservationBuffer::transformPoint( std::string new_global_frame,  const arc::PointStamped origin_in,  arc::PointStamped origin_out,  float R[9],   float T[3])
{
    //此处需要amcl出来的/map到robot的TF，这里先模拟写死
	 R[9]= (1.0, 0.0, 0.0, 0.0, 1.0 ,0.0 , 0.0, 0.0 ,1.0);
	 T[3]= (0.0, 0.0 ,1.0);
    if ( origin_in.header.frame_id == new_global_frame)
    {
        origin_out=origin_in;
    }
    else
    {
      float x=origin_in.point.x;
      float y=origin_in.point.y;
      float z=origin_in.point.z;
	  origin_out.point.x = R[0] * x + R[1] * y + R[2] * z + T[0];
	  origin_out.point.y = R[3] * x + R[4] * y + R[5] * z + T[1];
	  origin_out.point.z = R[6] * x + R[7] * y + R[8] * z + T[2];
	  origin_out.header.frame_id=new_global_frame;
    }
}
*/
}  // namespace costmap_2d




