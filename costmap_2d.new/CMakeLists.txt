cmake_minimum_required(VERSION 2.8)
project(costmap_2d.new)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
#SET(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
message(STATUS "[main]:cmake_prefix_path:" ${CMAKE_PREFIX_PATH})
message(STATUS "[main]:cmake_module_path:" ${CMAKE_MODULE_PATH})


set(CMAKE_PREFIX_PATH 
  /home/zach/noros_ws/devel
  /opt/ros/indigo
  /usr/local/opencv2
  )

find_package(catkin REQUIRED 
  COMPONENTS 
    tf 
    roscpp 
    nav_msgs 
    std_msgs
    pcl_ros
    pcl_conversions
    )
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS core Geometry)
find_package(OpenCV 2.4.8 REQUIRED)

message(STATUS "Boost find : " ${Boost_FOUND} )
message(STATUS "eigen3 find : " ${EIGEN3_FOUND} )
message(STATUS "opencv find : " ${OpenCV_FOUND} )


message(STATUS "Boost include dirs : " ${Boost_INCLUDE_DIR} )
message(STATUS "eigen3 include dirs : " ${EIGEN3_INCLUDE_DIR} )
message(STATUS "opencv include dirs : " ${OpenCV_INCLUDE_DIRS} )

message(STATUS "Boost libs : " ${Boost_LIBRARY_DIRS} )
message(STATUS "eigen3 libs : " ${EIGEN3_LIBRARIES} )
message(STATUS "opencv libs : " ${OpenCV_LIBS} )



add_library(costmap_2d STATIC 
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
  src/inflation_layer.cpp
  src/obstacle_layer.cpp
  src/static_layer.cpp
  src/observation_buffer.cpp
  src/costmap_settings.cpp
  )


INCLUDE_DIRECTORIES( 
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

target_link_libraries(costmap_2d
    ${Boost_LIBRARY_DIRS}
    ${EIGEN3_LIBRARIES} 
    ${OpenCV_INCLUDE_DIRS} 
    ${catkin_LIBRARIES}
    )



