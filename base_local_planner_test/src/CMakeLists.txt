set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread)
FIND_PACKAGE(Eigen3 REQUIRED)

MESSAGE("DEBUG eigen3include_dir: ${EIGEN3_INCLUDE_DIR}")

include_directories(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    /home/lixiaopeng/pepper/pepper_noros/costmap_2d_test/include
)
link_directories( /home/lixiaopeng/pepper/pepper_noros/costmap_2d_test/lib)

ADD_LIBRARY(base_local_planner_test SHARED
	footprint_helper.cpp 
        goal_functions.cpp
	map_cell.cpp
	map_grid.cpp
	map_grid_cost_function.cpp
	latched_stop_rotate_controller.cpp
	local_planner_util.cpp
	odometry_helper_ros.cpp
        obstacle_cost_function.cpp
        oscillation_cost_function.cpp
        prefer_forward_cost_function.cpp
        point_grid.cpp
        costmap_model.cpp
        simple_scored_sampling_planner.cpp
        simple_trajectory_generator.cpp
        trajectory.cpp
       )

include_directories(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

find_package(PkgConfig)


