#ifndef COSTMAP_2D_COSTMAP_MATH_H_
#define COSTMAP_2D_COSTMAP_MATH_H_

#include <math.h>
#include <algorithm>
#include <vector>
#include <costmap_2d_test/header.h>

inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

bool intersects(std::vector<arc::Point>& polygon, float testx, float testy);

bool intersects(std::vector<arc::Point>& polygon1, std::vector<arc::Point>& polygon2);

#endif  // COSTMAP_2D_COSTMAP_MATH_H_
