#ifndef COSTMAP_2D_FOOTPRINT_H
#define COSTMAP_2D_FOOTPRINT_H

#include <vector>
#include <costmap_2d_test/header.h>

namespace costmap_2d_test
{

void calculateMinAndMaxDistances(const std::vector<arc::Point>& footprint,
                                 double& min_dist, double& max_dist);

void transformFootprint(double x, double y, double theta, const std::vector<arc::Point>& footprint_spec,
                        std::vector<arc::Point>& oriented_footprint);

void transformFootprint(double x, double y, double theta, const std::vector<arc::Point>& footprint_spec,
                        arc::PolygonStamped & oriented_footprint);


void padFootprint(std::vector<arc::Point>& footprint, double padding);

std::vector<arc::Point> padrobotFootprint(std::vector<arc::Point>& footprint, double padding);


std::vector<arc::Point> makeFootprintFromRadius(double radius);


bool makeFootprintFromString(const std::string& footprint_string, std::vector<arc::Point>& footprint);


}  // end namespace costmap_2d

#endif  // COSTMAP_2D_FOOTPRINT_H
