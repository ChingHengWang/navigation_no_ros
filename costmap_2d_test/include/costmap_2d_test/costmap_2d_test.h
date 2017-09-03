#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <stdio.h>
#include <vector>
#include <queue>
#include <boost/thread.hpp>
#include <costmap_2d_test/observation_buffer.h>
#include <costmap_2d_test/header.h>
#include <costmap_2d_test/map_loader.h>
#include <costmap_2d_test/cost_values.h>
#include <costmap_2d_test/footprint.h>

namespace costmap_2d_test
{

struct MapLocation
{
    unsigned int x;
    unsigned int y;
};

class CellData
{
public:
    CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
    {
    }
    double distance_;
    unsigned int index_;
    unsigned int x_, y_;
    unsigned int src_x_, src_y_;
};

inline bool operator<(const CellData &a, const CellData &b)
{
    return a.distance_ > b.distance_;
}
class Costmap2D_Test
{
    friend class CostmapTester;  // Need this for gtest to work correctly
public:
    Costmap2D_Test(const Costmap2D_Test& map);
    Costmap2D_Test& operator=(const Costmap2D_Test& map);
    bool copyCostmapWindow(const Costmap2D_Test& map, double win_origin_x, double win_origin_y, double win_size_x, double win_size_y);
    Costmap2D_Test();
    inline unsigned char computeCost(double distance) const
    {
        unsigned char cost = 0;
        if (distance == 0)
            cost = LETHAL_OBSTACLE;
        else if (distance * resolution_ <= inscribed_radius_)
            cost = INSCRIBED_INFLATED_OBSTACLE;
        else
        {
            // make sure cost falls off by Euclidean distance
            double euclidean_distance = distance * resolution_;
            double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
            cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
    }
    virtual bool isDiscretized()
    {
        return true;
    }
    virtual ~Costmap2D_Test();
    void updatestaticMap(double robot_x, double robot_y, double robot_yaw);
    void updateinflationMap(double robot_x, double robot_y, double robot_yaw);
    void updateMap();
    unsigned char getCost(unsigned int mx, unsigned int my) const;
    void setCost(unsigned int mx, unsigned int my, unsigned char cost);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;
    void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;
    bool getRobotPose(double& robot_x, double& robot_y, double& robot_yaw) const;
    void updatestaticBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    void updatestaticCosts(int min_i, int min_j, int max_i, int max_j);
    void loadparam();
    void incomingMap(std::string file_name);
    unsigned char interpretValue(unsigned char value);
    void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
    void setInflationParameters(double inflation_radius, double cost_scaling_factor);
    void computeCaches();
    void deleteKernels();
    void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);
    double getInscribedRadius() { return inscribed_radius_; }
    double getCircumscribedRadius() { return circumscribed_radius_; }
    void setFootprint(const std::vector<arc::Point>& footprint_spec);
    const std::vector<arc::Point>& getFootprint() { return footprint_; }
    const std::vector<arc::Point>& getRobotFootprint() { return footprint_; }
    void updateinflationBounds(double robot_x, double robot_y, double robot_yaw, double min_x, double min_y, double max_x, double max_y);
    void updateinflationCosts(int min_i, int min_j, int max_i, int max_j);
    inline void enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y);
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }
    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / size_x_;
        mx = index - (my * size_x_);
    }
    unsigned char* getCharMap() const;
    unsigned int getSizeInCellsX() const;
    unsigned int getSizeInCellsY() const;
    double getSizeInMetersX() const;
    double getSizeInMetersY() const;
    double getOriginX() const;
    double getOriginY() const;
    double getResolution() const;
    void setDefaultValue(unsigned char c)
    {
        default_value_ = c;
    }
    unsigned char getDefaultValue()
    {
        return default_value_;
    }
    bool setConvexPolygonCost(const std::vector<arc::Point>& polygon, unsigned char cost_value);
    void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
    void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
    virtual void updateOrigin(double new_origin_x, double new_origin_y);
    bool saveMap(std::string file_name);
    void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);
    void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
    unsigned int cellDistance(double world_dist);
    void matchSize();

    // Provide a typedef to ease future code maintenance
    typedef boost::recursive_mutex mutex_t;
    mutex_t* getMutex()
    {
        return access_;
    }
protected:
    template<typename data_type>
    void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y, unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,unsigned int region_size_y)
    {
        // we'll first need to compute the starting points for each map
        data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
        data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);
        // now, we'll copy the source map into the destination map
        for (unsigned int i = 0; i < region_size_y; ++i)
        {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            sm_index += sm_size_x;
            dm_index += dm_size_x;
        }
    }
    void onFootprintChanged();
    /**
     * @brief  Deletes the costmap, static_map, and markers data structures
     */
    virtual void deleteMaps();
    bool getMarkingObservations(std::vector<costmap_2d_test::Observation>& marking_observations) const;
    bool getClearingObservations(std::vector<costmap_2d_test::Observation>& clearing_observations) const;
    void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void resetMaps();
    virtual void initMaps(unsigned int size_x, unsigned int size_y);
    template<class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX)
    {
        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * size_x_;

        unsigned int offset = y0 * size_x_ + x0;

        // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
        double dist = hypot(dx, dy);
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        // if x is dominant
        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;
            bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
            return;
        }

        // otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }
    boost::recursive_mutex* inflation_access_;
private:
    template<class ActionType>
    inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length)
    {
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }
    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }
    inline double distanceLookup(int mx, int my, int src_x, int src_y)
    {
	  //printf("update07 Infation Map Costs\n");
	  unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
  // printf("dx=%d dy=%d\n",dx,dy);
    return cached_distances_[dx][dy];
   // printf("update08 Infation Map Costs\n");
    }
    inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
    }
     std::string sensor_frame;
    mutex_t* access_;
    unsigned char lethal_threshold_, unknown_cost_value_,lethal_cost_threshold_;
    double minx_, miny_, maxx_, maxy_;
    bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot
    bool current_;
    bool has_extra_bounds_;
    bool enabled_;

    bool combination_method_;
    double cost_scaling_factor;
    std::string new_path;
    double  extra_min_x_,extra_min_y_,extra_max_x_,extra_max_y_;
    FILE* fp,*fp1,*fp2,*fp3;
    std::vector<arc::Point> transformed_footprint_;
protected:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;
    double max_obstacle_height_;
    unsigned int x_, y_, width_, height_;
    bool map_received_;
    bool has_updated_data_;
    double radius;
    double padding;
    bool track_unknown_space_,trinary_costmap_;
    bool use_maximum_;
    int x0,y0,xn,yn;          //update map area
    std::string global_frame_;
    std::vector<costmap_2d_test::Observation> static_clearing_observations_, static_marking_observations_;
    bool footprint_clearing_enabled_;
    std::vector<arc::Point> footprint_;
    double inflation_radius_, inscribed_radius_, weight_,inflation_radius;
    unsigned int cell_inflation_radius_;
    unsigned int cached_cell_inflation_radius_;
    std::priority_queue<CellData> inflation_queue_;
    bool* seen_;
    int seen_size_;
    unsigned char** cached_costs_;
    double** cached_distances_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    double circumscribed_radius_;
    std::vector<arc::Point> robot_footprint;
    std::vector<arc::Point> robot_padding_footprint;
    bool stop_updates_;
    class MarkCell
    {
    public:
        MarkCell(unsigned char* costmap, unsigned char value) :
            costmap_(costmap), value_(value)
        {
        }
        inline void operator()(unsigned int offset)
        {
            costmap_[offset] = value_;
        }
    private:
        unsigned char* costmap_;
        unsigned char value_;
    };
    class PolygonOutlineCells
    {
    public:
        PolygonOutlineCells(const Costmap2D_Test& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
            costmap_(costmap), char_map_(char_map), cells_(cells)
        {
        }

        // just push the relevant cells back onto the list
        inline void operator()(unsigned int offset)
        {
            MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            cells_.push_back(loc);
        }

    private:
        const Costmap2D_Test& costmap_;
        const unsigned char* char_map_;
        std::vector<MapLocation>& cells_;
    };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H

