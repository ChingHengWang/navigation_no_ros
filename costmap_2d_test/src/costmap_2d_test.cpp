#include <costmap_2d_test/costmap_2d_test.h>
#include <cstdio>
#include <cv_debug_header.h>

using namespace std;
using costmap_2d_test::NO_INFORMATION;
using costmap_2d_test::LETHAL_OBSTACLE;
using costmap_2d_test::FREE_SPACE;

using costmap_2d_test::ObservationBuffer;
using costmap_2d_test::Observation;

namespace costmap_2d_test
{
Costmap2D_Test::Costmap2D_Test() : default_value_(0),size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL),cell_inflation_radius_(0), cached_cell_inflation_radius_(0),seen_(NULL),cached_costs_(NULL),cached_distances_(NULL)
    , last_min_x_(-std::numeric_limits<float>::max())
    , last_min_y_(-std::numeric_limits<float>::max())
    , last_max_x_(std::numeric_limits<float>::max())
    , last_max_y_(std::numeric_limits<float>::max())
    ,weight_(0)
{
    //static map
    int temp_lethal_threshold, temp_unknown_cost_value;
    track_unknown_space_=true;
    trinary_costmap_=true;
    use_maximum_=false;
    temp_lethal_threshold=100;
    temp_unknown_cost_value=-1;
    unknown_cost_value_ = temp_unknown_cost_value;
    lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
    map_received_ = false;
    has_updated_data_ = false;
    //obstacle map
    rolling_window_=false;
    enabled_=true;
    if (seen_)
        delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    stop_updates_=false;
    loadparam();
   // incomingMap();
    setInflationParameters(inflation_radius, cost_scaling_factor);
}
void Costmap2D_Test::deleteMaps()
{
    // clean up data
    delete[] costmap_;
    costmap_ = NULL;
}
Costmap2D_Test::~Costmap2D_Test()
{
    deleteMaps();
}
void Costmap2D_Test::initMaps(unsigned int size_x, unsigned int size_y)
{
    delete[] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
}
void Costmap2D_Test::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    initMaps(size_x, size_y);
    // reset our maps to have no information
    resetMaps();
}
void Costmap2D_Test::resetMaps()
{
    //  boost::unique_lock<mutex_t> lock(*access_);
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}
void Costmap2D_Test::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
    //  boost::unique_lock<mutex_t> lock(*(access_));
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
        memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}
bool Costmap2D_Test::copyCostmapWindow(const Costmap2D_Test& map, double win_origin_x, double win_origin_y, double win_size_x, double win_size_y)
{
    // check for self windowing
    if (this == &map)
    {
        return false;
    }
    deleteMaps();
    // compute the bounds of our new map
    unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y))
    {
        return false;
    }
    size_x_ = upper_right_x - lower_left_x;
    size_y_ = upper_right_y - lower_left_y;
    resolution_ = map.resolution_;
    origin_x_ = win_origin_x;
    origin_y_ = win_origin_y;
    // initialize our various maps and reset markers for inflation
    initMaps(size_x_, size_y_);
    // copy the window of the static map and the costmap that we're taking
    copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
    return true;
}
Costmap2D_Test& Costmap2D_Test::operator=(const Costmap2D_Test& map)
{
    // check for self assignement
    if (this == &map)
        return *this;
    // clean up old data
    deleteMaps();
    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;
    // initialize our various maps
    initMaps(size_x_, size_y_);
    // copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    return *this;
}
Costmap2D_Test::Costmap2D_Test(const Costmap2D_Test& map) :
    costmap_(NULL)
{
    *this = map;
}
unsigned int Costmap2D_Test::cellDistance(double world_dist)
{
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int)cells_dist;
}
unsigned char* Costmap2D_Test::getCharMap() const
{
    return costmap_;
}
unsigned char Costmap2D_Test::getCost(unsigned int mx, unsigned int my) const
{
    return costmap_[getIndex(mx, my)];
}
void Costmap2D_Test::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    costmap_[getIndex(mx, my)] = cost;
}
bool Costmap2D_Test::getRobotPose(double& robot_x, double& robot_y, double& robot_yaw) const
{
    robot_x=1.0;
    robot_y=1.0;
    robot_yaw=0.0;
    return true;
}
void Costmap2D_Test::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}
bool Costmap2D_Test::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
    if (mx < size_x_ && my < size_y_)
        return true;
    return false;
}
void Costmap2D_Test::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
}
void Costmap2D_Test::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
    // Here we avoid doing any math to wx,wy before comparing them to
    // the bounds, so their values can go out to the max and min values
    // of double floating point.
    if (wx < origin_x_)
    {
        mx = 0;
    }
    else if (wx > resolution_ * size_x_ + origin_x_)
    {
        mx = size_x_ - 1;
    }
    else
    {
        mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_)
    {
        my = 0;
    }
    else if (wy > resolution_ * size_y_ + origin_y_)
    {
        my = size_y_ - 1;
    }
    else
    {
        my = (int)((wy - origin_y_) / resolution_);
    }
}
void Costmap2D_Test::updateOrigin(double new_origin_x, double new_origin_y)
{
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);
    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;
    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;
    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);
    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;
    // we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    // copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
    // now we'll set the costmap to be completely unknown if we track unknown space
    resetMaps();
    // update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;
    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;
    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
    // make sure to clean up
    delete[] local_map;
}
bool Costmap2D_Test::setConvexPolygonCost(const std::vector<arc::Point>& polygon, unsigned char cost_value)
{
    // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
    std::vector<MapLocation> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i)
    {
        MapLocation loc;
        if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
        {
            return false;
        }
        map_polygon.push_back(loc);
    }
    std::vector<MapLocation> polygon_cells;
    // get the cells that fill the polygon
    convexFillCells(map_polygon, polygon_cells);
    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i)
    {
        unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
        costmap_[index] = cost_value;
    }
    return true;
}
void Costmap2D_Test::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
    for (unsigned int i = 0; i < polygon.size() - 1; ++i)
    {
        raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
    }
    if (!polygon.empty())
    {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
}
void Costmap2D_Test::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
    // we need a minimum polygon of a triangle
    if (polygon.size() < 3)
        return;
    // first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);
    // quick bubble sort to sort points by x
    MapLocation swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1)
    {
        if (polygon_cells[i].x > polygon_cells[i + 1].x)
        {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;
            if (i > 0)
                --i;
        }
        else
            ++i;
    }
    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;
    // walk through each column and mark cells inside the polygon
    for (unsigned int x = min_x; x <= max_x; ++x)
    {
        if (i >= polygon_cells.size() - 1)
            break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y)
        {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        }
        else
        {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }
        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x)
        {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
            ++i;
        }
        MapLocation pt;
        // loop though cells in the column
        for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
        {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}
unsigned int Costmap2D_Test::getSizeInCellsX() const
{
    return size_x_;
}
unsigned int Costmap2D_Test::getSizeInCellsY() const
{
    return size_y_;
}
double Costmap2D_Test::getSizeInMetersX() const
{
    return (size_x_ - 1 + 0.5) * resolution_;
}
double Costmap2D_Test::getSizeInMetersY() const
{
    return (size_y_ - 1 + 0.5) * resolution_;
}
double Costmap2D_Test::getOriginX() const
{
    return origin_x_;
}
double Costmap2D_Test::getOriginY() const
{
    return origin_y_;
}
double Costmap2D_Test::getResolution() const
{
    return resolution_;
}
bool Costmap2D_Test::saveMap(std::string file_name)
{
    FILE *fp = fopen(file_name.c_str(), "w");

    if (!fp)
    {
        return false;
    }

    fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
    for (unsigned int iy = 0; iy < size_y_; iy++)
    {
        for (unsigned int ix = 0; ix < size_x_; ix++)
        {
            unsigned char cost = getCost(ix, iy);
            fprintf(fp, "%d ", cost);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}
unsigned char Costmap2D_Test::interpretValue(unsigned char value)
{
    // check if the static value is above the unknown or lethal thresholds
    //printf("track_unknown_space : %d %d\n",track_unknown_space_, unknown_cost_value_);
    if (track_unknown_space_ && value == unknown_cost_value_)
        return NO_INFORMATION;
    else if (!track_unknown_space_ && value == unknown_cost_value_)
        return FREE_SPACE;
    else if (value >= lethal_threshold_)
        return LETHAL_OBSTACLE;
    else if (trinary_costmap_)
        return FREE_SPACE;
    double scale = (double) value / lethal_threshold_;
    return scale * LETHAL_OBSTACLE;
}
void Costmap2D_Test::matchSize()
{
    resolution_ = getResolution();
    printf("inflation_radius_=%f\n",inflation_radius_);
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    printf("cell_inflation_radius=%d\n",cell_inflation_radius_);
    computeCaches();
    unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();
    printf("size_x=%d size_y=%d\n",size_x,size_y);
    if(seen_)
        delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
    printf("matchsize\n");
}
void Costmap2D_Test::loadparam()
{
	arc::ParameterReader  pr;
    radius =   atof( pr.getData( "robot_radius" ).c_str() );
    printf("radius=%f\n",radius);
    cost_scaling_factor=atof( pr.getData( "cost_scaling_factor" ).c_str() );
    padding=atof( pr.getData( "padding" ).c_str() );
    inflation_radius=atof( pr.getData( "inflation_radius" ).c_str() );
    max_obstacle_height_=atof( pr.getData( "max_obstacle_height_" ).c_str() );
    printf("max_obstacle_height_=%f\n",  max_obstacle_height_);
    int radius_=atof( pr.getData( "radius_" ).c_str() );
   printf("radius_=%d\n",radius_);
   new_path=pr.getData( "path" ).c_str();
   printf("path=%s\n",new_path.c_str());
   incomingMap(new_path.c_str());

}
void Costmap2D_Test::incomingMap(std::string  file_name)
{
    printf("incoming map\n");
    MapLoader ml(file_name);
    arc::OccupancyGrid new_map;
    ml.load(new_map);

    
    /*debug*/
    #if 1
    cv::Mat M=cv::Mat(new_map.info.height,new_map.info.width,CV_8UC1);
    memcpy(M.data,new_map.data.data(),new_map.data.size()*sizeof(int8_t));
    //cv::Mat M_cv=cv::Mat(new_map.info.height,new_map.info.width,CV_8UC1);
    //createOpenCVDebugMat(M,M_cv);
    imshow("nav_ws incomingMap", M);
    cvWaitKey();
    #endif
    /*debug*/

    
    unsigned int size_x = new_map.info.width, size_y = new_map.info.height;
    printf("Size_X=%d Size_Y=%d resolution=%f\n",size_x,size_y,new_map.info.resolution);
    resizeMap(size_x, size_y, new_map.info.resolution, new_map.info.origin.position.x, new_map.info.origin.position.y);
    unsigned int index = 0;
    for (unsigned int i = 0; i < size_y; ++i)
    {
        for (unsigned int j = 0; j < size_x; ++j)
        {
            unsigned char value = new_map.data[index];
        	//int8_t value = new_map.data[index];
           costmap_[index] = interpretValue(value);
            ++index;
        }
    }
    /*debug*/
    #if 1
    cv::Mat M_tmp=cv::Mat(size_y,size_x,CV_8UC1);
    memcpy(M_tmp.data,costmap_,size_x*size_y*sizeof(int8_t));
    imshow("costmap_ noros", M_tmp);
    cvWaitKey();
    #endif
    /*debug*/

 
    
    x_=y_=0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_=true;
  has_updated_data_ = true;
}
void Costmap2D_Test::updatestaticMap(double robot_x, double robot_y, double robot_yaw)
{
	printf("ENTER STATIC LAYER\n");
	updatestaticBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, & maxx_, &maxy_);
    worldToMapEnforceBounds(minx_, miny_, x0, y0);
    worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
    x0 = std::max(0, x0);
    xn = std::min(int(getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(int(getSizeInCellsY()), yn + 1);
    printf("x0=%d xn=%d y0=%d yn=%d\n",x0,xn,y0,yn);
    if (xn < x0 || yn < y0)
        return;
}
void Costmap2D_Test::updateinflationMap(double robot_x, double robot_y, double robot_yaw)
{
    updateinflationBounds(robot_x, robot_y, robot_yaw, minx_, miny_, maxx_, maxy_);
    worldToMapEnforceBounds(minx_, miny_, x0, y0);
    worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
    x0 = std::max(0, x0);
    xn = std::min(int(getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(int(getSizeInCellsY()), yn + 1);
    printf("x0=%d xn=%d y0=%d yn=%d\n",x0,xn,y0,yn);
    if (xn < x0 || yn < y0)
      return;
    updateinflationCosts(x0, y0, xn, yn);
}
void Costmap2D_Test::updateMap()
{
    if(!stop_updates_)
    {
        double robot_x,robot_y,robot_yaw;
        long t_diff;
        struct timeval t_start,t_end;
        gettimeofday(&t_start, NULL);
        long start0 = ((long)t_start.tv_sec)*1000+(long)t_start.tv_usec/1000;
        printf("Start time: %ld ms\n", start0);
        if(getRobotPose(robot_x, robot_y, robot_yaw))
        {
        	printf("start update map!\n");
            printf("robot_x=%f robot_y=%f robot_yaw=%f\n",robot_x,robot_y,robot_yaw);
            extra_min_x_=extra_min_y_=1e6;
            extra_max_x_=extra_max_y_=-1e6;
            minx_ = miny_ = 1e30;
            maxx_ = maxy_ = -1e30;
            double prev_minx = minx_;
            double prev_miny = miny_;
            double prev_maxx = maxx_;
            double prev_maxy = maxy_;
            updatestaticMap( robot_x, robot_y, robot_yaw);
            printf("Sx0=%d Sxn=%d Sy0=%d Syn=%d\n",x0,xn,y0,yn);
            updateFootprint(robot_x,  robot_y,  robot_yaw,  &minx_,  &miny_,  &maxx_,  &maxy_);
            setFootprint(transformed_footprint_);
            setConvexPolygonCost(transformed_footprint_, costmap_2d_test::FREE_SPACE);
            //updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
            matchSize();
            printf("ix0=%d ixn=%d iy0=%d iyn=%d\n",x0,xn,y0,yn);
            updateinflationMap(robot_x,  robot_y,  robot_yaw);
            printf("Sx0=%d Sxn=%d Sy0=%d Syn=%d\n",x0,xn,y0,yn);
        }
        gettimeofday(&t_end, NULL);
        long end0= ((long)t_end.tv_sec)*1000+(long)t_end.tv_usec/1000;
        printf("Start time: %ld ms\n", end0);
        t_diff=end0-start0;
        printf("Map update time: %ld ms\n", t_diff);
    }
}
void Costmap2D_Test::updatestaticBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    printf("Update Static Map Bounds!\n");
    if( !rolling_window_ )
    {
        if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
            return;
    }
    printf("Update Static Map Bounds!\n");
  //  useExtraBounds(min_x, min_y, max_x, max_y);
    double wx, wy;
    mapToWorld(x_, y_, wx, wy);
    printf("x_=%d y_=%d wx=%f wy=%f\n",x_,y_,wx,wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);
    mapToWorld(x_ + width_, y_ + height_, wx, wy);
    printf("x_=%d y_=%d wx=%f wy=%f\n",x_+width_,y_+height_,wx,wy);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);
    printf("minx=%f miny=%f maxx=%f maxy=%f\n",*min_x,*min_y,*max_x,*max_y);
    //has_updated_data_ = false;
}
void Costmap2D_Test::updateinflationBounds(double robot_x, double robot_y, double robot_yaw, double min_x, double min_y, double max_x, double max_y)
{
      printf("Update Inflation Bounds\n");
      minx_ = minx_ - inflation_radius_;
      miny_ = miny_ - inflation_radius_;
      maxx_ = maxx_ + inflation_radius_;
      maxy_ = maxy_ + inflation_radius_;
}
void Costmap2D_Test::updateinflationCosts(int min_i, int min_j, int max_i, int max_j)
{
    printf("update Infation Map Costs\n");
    if (!enabled_)
        return;
    // make sure the inflation queue is empty at the beginning of the cycle (should always be true)
    unsigned char* master_array = getCharMap();
    unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();
    if (seen_ == NULL)
    {
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    else if (seen_size_ != size_x * size_y)
    {
        delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
        printf("update10 Infation Map Costs\n");
    }
    memset(seen_, false, size_x * size_y * sizeof(bool));
    // We need to include in the inflation cells outside the bounding
    // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
    // up to that distance outside the box can still influence the costs
    // stored in cells inside the box.
    min_i -= cell_inflation_radius_;
    min_j -= cell_inflation_radius_;
    max_i += cell_inflation_radius_;
    max_j += cell_inflation_radius_;
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(int(size_x), max_i);
    max_j = std::min(int(size_y), max_j);
    printf("mini=%d minj=%d maxi=%d maxj=%d\n",min_i,min_j,max_i,max_j);
    printf("update0 Infation Map Costs\n");

    #define debug_lethal_queue true
    #if debug_lethal_queue
    cv::Mat M_tmp=cv::Mat(size_y,size_x,CV_8UC1);
    cv::Point pt;
    #endif
    for (int j = min_j; j < max_j; j++)
    {
        for (int i = min_i; i < max_i; i++)
        {
            int index = getIndex(i, j);
            //printf("update00 Infation Map Costs\n");
            unsigned char cost = master_array[index];
            //  printf("m=%c\n",master_array[index]);
            //  printf("update01 Infation Map Costs\n");
            if (cost == LETHAL_OBSTACLE)
            {
               // printf("update02 Infation Map Costs\n");
                enqueue(index, i, j, i, j);
              // printf("update03 Infation Map Costs\n");
                #if debug_lethal_queue
                pt.x = i;
                pt.y = j;
                cv::circle(M_tmp, pt, 1, cv::Scalar(255,255,255));
                #endif
            }
        }
    }
    #if debug_lethal_queue
    cv::imshow("lethal obstacle", M_tmp);
    cv::waitKey();
    #endif
 
    cv::Mat M_tmp2=cv::Mat(size_y,size_x,CV_8UC1);
    cv::Point pt2;
    int count = 0;
        // printf("inflation_queue_=%d\n",inflation_queue_.size());
    while (!inflation_queue_.empty())
    {
        const CellData& current_cell = inflation_queue_.top();
        unsigned int index = current_cell.index_;
        unsigned int mx = current_cell.x_;
        unsigned int my = current_cell.y_;
        unsigned int sx = current_cell.src_x_;
        unsigned int sy = current_cell.src_y_;
        //printf("inflaiton_queue_top mx %d sx %d my %d sy %d\n",mx,sx,my,sy);
        inflation_queue_.pop();
        // set the cost of the cell being inserted
        if (seen_[index])
        {
            continue;
        }
        seen_[index] = true;
        // assign the cost associated with the distance from an obstacle to the cell
        unsigned char cost = costLookup(mx, my, sx, sy);
        unsigned char old_cost = master_array[index];
        if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
            costmap_[index] = cost;
        else
            costmap_[index] = std::max(old_cost, cost);
        // attempt to put the neighbors of the current cell onto the queue
        if (mx > 0)
            enqueue(index - 1, mx - 1, my, sx, sy);
        if (my > 0)
            enqueue(index - size_x, mx, my - 1, sx, sy);
        if (mx < size_x - 1)
            enqueue(index + 1, mx + 1, my, sx, sy);
        if (my < size_y - 1)
            enqueue(index + size_x, mx, my + 1, sx, sy);


        #if 1
        if(count%200 == 0){
            memcpy(M_tmp2.data,costmap_,size_y*size_x*sizeof(int8_t));
            imshow("inflation costmap ongoing", M_tmp2);
            cvWaitKey(10);
            printf("cvWaitKey count %d\n",count);
        }
    #endif
        count++; 
    }
  
    
//            unsigned int index3 = 0;
//            unsigned int X3;
//            unsigned int Y3;
//            unsigned char* master3;
//            X3=getSizeInCellsX();
//            Y3=getSizeInCellsY();
//            printf("X=%d Y=%d\n",X3,Y3);
//            master3=getCharMap();
//            for (unsigned int i = 0; i <X3; ++i)
//            {
//                for (unsigned int j = 0; j <Y3; ++j)
//                {
//                    ++index3;
//                    fprintf(fp2,"%d\n",master3[index3]);//把cost数据打印到data.txt中
//                }
//            }
}
void Costmap2D_Test::updatestaticCosts(int min_i, int min_j, int max_i, int max_j)
{
    printf("Update Static Map Costs\n");
    if (!map_received_)
        return;
    if (!rolling_window_)
    {
        // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
//            unsigned int index1 = 0;
//            unsigned int X1;
//            unsigned int Y1;
//            unsigned char* master1;
//            X1=getSizeInCellsX();
//            Y1=getSizeInCellsY();
//            printf("X1=%d Y1=%d\n",X1,Y1);
//            master1=getCharMap();
//            for (unsigned int i = 0; i <X1; ++i)
//            {
//                for (unsigned int j = 0; j <Y1; ++j)
//                {
//                    ++index1;
//                    fprintf(fp1,"%d\n",master1[index1]);//把cost数据打印到data.txt中
//                }
//            }
//            printf("save static costmap\n");
    }
    else
    {
        // If rolling window, the master_grid is unlikely to have same coordinates as this layer
        unsigned int mx, my;
        double wx, wy;
        // Copy map data given proper transformations
        for (unsigned int i = min_i; i < max_i; ++i)
        {
            for (unsigned int j = min_j; j < max_j; ++j)
            {
                // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
                mapToWorld(i, j, wx, wy);
                if (worldToMap(wx, wy, mx, my))
                {
                    if (!use_maximum_)
                        setCost(i, j, getCost(mx, my));
                    else
                        setCost(i, j, std::max(getCost(mx, my), getCost(i, j)));
                }
            }
        }
    }
}
void Costmap2D_Test::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    printf("Update robot footprint!\n");
    robot_footprint=makeFootprintFromRadius(radius);
    robot_padding_footprint=padrobotFootprint(robot_footprint, padding);
    transformFootprint(robot_x, robot_y, robot_yaw, robot_padding_footprint, transformed_footprint_);
    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
        touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
    printf("minx=%f miny=%f maxx=%f maxy=%f\n",*min_x,*min_y,*max_x,*max_y);
}
void Costmap2D_Test::touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
}
void Costmap2D_Test::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
    if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
    {
        // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
        // when accessing the cached arrays
        //   boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
        inflation_radius_ = inflation_radius;
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        weight_ = cost_scaling_factor;
        computeCaches();
    }
}
inline void Costmap2D_Test::enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y)
{
    //printf("update04 Infation Map Costs\n");
    if (!seen_[index])
    {
        // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
      // printf("update05 Infation Map Costs\n");
        double distance = distanceLookup(mx, my, src_x, src_y);
     //  printf("update06 Infation Map Costs\n");
        // we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
        if (distance > cell_inflation_radius_)
            return;
        // push the cell data onto the queue and mark
        CellData data(distance, index, mx, my, src_x, src_y);
        inflation_queue_.push(data);
    }
}
void Costmap2D_Test::computeCaches()
{
   // printf("computeCaches\n");
    if (cell_inflation_radius_ == 0)
        return;
    // based on the inflation radius... compute distance and cost caches
    if (cell_inflation_radius_ != cached_cell_inflation_radius_)
    {
        printf("computeCaches0\n");
        deleteKernels();
        printf("computeCaches\n");
        cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
        cached_distances_ = new double*[cell_inflation_radius_ + 2];
        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_distances_[i][j] = hypot(i, j);
            }
        }
        cached_cell_inflation_radius_ = cell_inflation_radius_;
        printf("cached_cell_inflation_radius_=%d\n",cached_cell_inflation_radius_);
    }
    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
        for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
        {
            cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
        }
    }
}
void Costmap2D_Test::deleteKernels()
{
    if (cached_distances_ != NULL)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_distances_[i])
                delete[] cached_distances_[i];
        }
        if (cached_distances_)
            delete[] cached_distances_;
        cached_distances_ = NULL;
    }
    if (cached_costs_ != NULL)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_costs_[i])
                delete[] cached_costs_[i];
        }
        delete[] cached_costs_;
        cached_costs_ = NULL;
    }
}
void Costmap2D_Test::onFootprintChanged()
{
    inscribed_radius_ = getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();
}
void Costmap2D_Test::setFootprint(const std::vector<arc::Point>& footprint_spec)
{
    footprint_ = footprint_spec;
    costmap_2d_test::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);
    onFootprintChanged();
}
}  // namespace costmap_2d



