#ifndef COSTMAP_SETTINGS_HPP
#define COSTMAP_SETTINGS_HPP

#include <string>
using namespace std;

namespace costmap_2d
{
    //for map loader
    extern string g_map_image;
    extern double g_resolution;
    extern int    g_negate;
    extern double g_occupied_thresh;
    extern double g_free_thresh;
    extern double g_origin_x;
    extern double g_origin_y;
    extern double g_origin_z;

    extern double g_pose_update_frequency;
    extern double g_map_update_frequency;
    extern bool g_static_map;

    //for costmap2DROS
    extern bool   g_rolling_window;
    extern bool   g_track_unknow_space;
    extern int    g_map_width;
    extern int    g_map_height;
    extern double g_map_resolution;
    extern double g_robot_radius;
    extern double g_footprint_padding;

    extern std::string g_global_frame;
    extern std::string g_robot_base_frame;

    //for map update
    extern double g_transform_tolerance;
    extern double g_obstacle_range;
    extern double g_raytrace_range;
    extern double g_map_max_obstacle_height;

    //for laser scan
    extern double g_laser_expected_update_rate;
    extern double g_laser_observation_persistence;
    extern bool   g_laser_marking;
    extern bool   g_laser_clearing;
    extern double g_laser_max_obstacle_height;
    extern double g_laser_min_obstacle_height;

    //for static layer
    extern bool   g_static_enabled;
    extern double g_static_lethal_cost_threshold;

    //for obstacle layer
    extern bool g_obstacle_enabled;
    extern bool g_obstacle_footprint_clearing_enabled;
    extern int  g_obstacle_combination_method;

    //for inflation layer
    extern bool   g_inflation_enabled;
    extern double g_inflation_radius;
    extern double g_cost_scaling_factor;
}




#endif
