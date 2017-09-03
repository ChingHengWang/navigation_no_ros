#include <costmap_2d_test/header.h>
#include <string>
#include <Eigen/Dense>

class pointcloud
{
public:
	pointcloud():header(),points(),Point()
        {
        };
        pointcloud( const pointcloud& pc)
        {
        	 *this = pc;
        };
        arc::Header header;
    	std::vector< Eigen::Vector3d > points;
        arc::PointCloud Point;
};
