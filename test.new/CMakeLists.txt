cmake_minimum_required(VERSION 2.8.3)
project(test)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS -std=c++11)	
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

set(CMAKE_PREFIX_PATH 
  /home/zach/noros_ws/devel
  /opt/ros/indigo
  /usr/local/opencv2
  )

message("CMAKE_PREFIX_PATH : ${CMAKE_PREFIX_PATH}")
find_package(X11 REQUIRED)
message("X11_INCLUDE_DIR : ${X11_INCLUDE_DIR}")
message("X11_LIBRARIES : ${X11_LIBRARIES}")
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs pcl_ros pcl_conversions)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)
#message(${OpenCV_INCLUDE_DIRS})

message(STATUS "Boost find : " ${Boost_FOUND} )
message(STATUS "eigen3 find : " ${EIGEN3_FOUND} )
message(STATUS "opencv find : " ${OpenCV_FOUND} )
message(STATUS "Boost include dirs : " ${Boost_INCLUDE_DIR} )
message(STATUS "eigen3 include dirs : " ${EIGEN3_INCLUDE_DIR} )
message(STATUS "opencv include dirs : " ${OpenCV_INCLUDE_DIRS} )
message(STATUS "Boost libs : " ${Boost_LIBRARIES}  )
message(STATUS "eigen3 libs : " ${EIGEN3_LIBRARIES} )
message(STATUS "opencv libs : " ${OpenCV_LIBRARIES} )



INCLUDE_DIRECTORIES( 
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /home/zach/noros_ws/navigation_no_ros.pc_version/pkg/costmap_2d.new/include
    ${catkin_INCLUDE_DIRS}
    ${X11_INCLUDE_DIR}    

    )
LINK_DIRECTORIES(
    /home/zach/noros_ws/navigation_no_ros.pc_version/pkg/costmap_2d.new/lib
    )


add_executable (pepper_debug src/test.cpp)
target_link_libraries(pepper_debug
    libcostmap_2d.a
    ${Boost_LIBRARIES}   
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${X11_LIBRARIES}

    )

