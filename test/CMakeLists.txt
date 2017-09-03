cmake_minimum_required(VERSION 2.8.3)
project(test)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS -std=c++11)	
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package( OpenCV 2.4.8 REQUIRED )
message(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs)
SET(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)


INCLUDE_DIRECTORIES( 
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /home/zach/noros_ws/navigation_no_ros.master/pkg/costmap_2d_test/include
    /home/zach/noros_ws/navigation_no_ros.master/pkg/my_navfn/include
    #    ${catkin_INCLUDE_DIRS}
    )
LINK_DIRECTORIES(
    /home/zach/noros_ws/navigation_no_ros.master/pkg/costmap_2d_test/lib
    /home/zach/noros_ws/navigation_no_ros.master/pkg/my_navfn/lib
 )

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

message(STATUS "include dirs : " ${CMAKE_CURRENT_SOURCE_DIR} )

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "link='${dir}'")
endforeach()


#message(STATUS "include dir: "${OPENCV_DIRS})	
#message(STATUS "include dir: "${PROJECT_ROOT_PATH})	
#message(STATUS "include dir: "${CMAKE_INCLUDE_PATH})	
#message(STATUS "include dir: "${CMAKE_LIBRARY_PATH})	
#message(STATUS "include dir: "${CMAKE_MODULE_PATH})	
#MESSAGE(STATUS "HOME dir: $ENV{HOME}")
#message(STATUS "include dir: "${CMAKE_BINARY_PATH})	
#message(STATUS "include dir: "${CMAKE_BUILD_TYPE})	
#message(STATUS "include dir: "${CMAKE_MODULE_PATH})	





add_executable (pepper_test src/test.cpp)
target_link_libraries(pepper_test
    libcostmap_2d_test.so
    libmy_navfn.so   
    ${Boost_LIBRARIES}   
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    SDL
    SDL_image
    ${OpenCV_LIBS}
    )

add_executable (pepper_gazebo_test src/gazebo_test.cpp)
target_link_libraries(pepper_gazebo_test
    libcostmap_2d_test.so
    libmy_navfn.so   
    ${Boost_LIBRARIES}   
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    SDL
    SDL_image
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)



