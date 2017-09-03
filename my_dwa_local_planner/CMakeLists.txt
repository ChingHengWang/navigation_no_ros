CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本
project(my_dwa_local_planner)
SET(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module ${CMAKE_MODULE_PATH})
#find_package(cmake_modules REQUIRED)


FIND_PACKAGE( OpenCV 2.4.8 REQUIRED )

find_package(Eigen3 REQUIRED)
SET(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)


#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /home/zach/nav_ws/pepperHost1/navigation_no_ros/pkg/costmap_2d_test/include
    /home/zach/nav_ws/pepperHost1/navigation_no_ros/pkg/base_local_planner_test/include
)

LINK_DIRECTORIES( 
    ${PROJECT_SOURCE_DIR}/lib
    /home/zach/nav_ws/pepperHost1/navigation_no_ros/pkg/costmap_2d_test/lib
    /home/zach/nav_ws/pepperHost1/navigation_no_ros/pkg/base_local_planner_test/lib

)

#include_directories(/home/zach/catkin_ws/src/andbot_simulator/navigation/debug/include)


add_library(my_dwa_local_planner SHARED
	src/my_dwa_planner.cpp 
	src/my_dwa_planner_wrapper.cpp 
)

target_link_libraries(my_dwa_local_planner 
	libbase_local_planner_test.so
	libcostmap_2d_test.so
)

message(STATUS "eigen3 include dir: ${EIGEN3_INCLUDE_DIRS}")	
#message(STATUS "project source dir: "${PROJECT_SOURCE_DIR})	
#message(STATUS "include dir: "${OPENCV_DIRS})	
#message(STATUS "include dir: "${PROJECT_ROOT_PATH})	
#message(STATUS "include dir: "${CMAKE_INCLUDE_PATH})	
#message(STATUS "include dir: "${CMAKE_LIBRARY_PATH})	
#message(STATUS "include dir: "${CMAKE_MODULE_PATH})	
#MESSAGE(STATUS "HOME dir: $ENV{HOME}")
#message(STATUS "include dir: "${CMAKE_BINARY_PATH})	
#message(STATUS "include dir: "${CMAKE_BUILD_TYPE})	
#message(STATUS "include dir: "${CMAKE_MODULE_PATH})	





