#include <costmap_2d/costmap_settings.h>

namespace costmap_2d
{
    //for map loader for global_costmap
    string g_map_image = "/home/zach/noros_ws/navigation_no_ros.pc_version/pkg/test.new/map/simple_maze.pgm";
    double g_resolution = 0.05;
    int    g_negate = 0;
    double g_occupied_thresh = 0.65;
    double g_free_thresh = 0.196;
    double g_origin_x = -5.0;
    double g_origin_y = -5.0;
    double g_origin_z = 0.0;

    double g_pose_update_frequency = 10.0;
    double g_map_update_frequency = 10.0;
    bool   g_static_map = true;

    //for costmap2DROS for local_costmap
    bool   g_rolling_window = true;
    //bool   g_track_unkonw_space = false;
    int    g_map_width = 16;
    int    g_map_height = 16;
    double g_map_resolution = 0.05;
    double g_robot_radius = 0.46;
    double g_footprint_padding = 0.01; 

    std::string g_global_frame = std::string("map");
    std::string g_robot_base_frame = std::string("xv11");

    //for map update
    double g_transform_tolerance = 0.3;
    double g_obstacle_range = 4.5;
    double g_raytrace_range = 5.0;
    double g_map_max_obstacle_height = 2.0;

    //for laser scan params
    double g_laser_expected_update_rate = 0.4;
    double g_laser_observation_persistence = 0.0;
    bool   g_laser_marking = true;
    bool   g_laser_clearing = true;
    double g_laser_max_obstacle_height = 0.4;
    double g_laser_min_obstacle_height = 0.08;

    //for static layer
    bool   g_static_enabled = true;
    double g_lethal_cost_threshold = 100;

    //for obstacle layer
    bool g_obstacle_enabled = true;
    bool g_obstacle_footprint_clearing_enabled = true;
    int  g_obstacle_combination_method = 1;//0;

    //for inflation layer
    bool   g_inflation_enabled = true;
    double g_inflation_radius = 3.0;
    double g_cost_scaling_factor = 0.8;
}
