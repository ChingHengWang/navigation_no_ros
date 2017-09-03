set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread)
FIND_PACKAGE(Eigen3 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)


MESSAGE("DEBUG eigen3 include_dir: ${EIGEN3_INCLUDE_DIR}")
MESSAGE("DEBUG opencv include_dir: ${OpenCV_INCLUDE_DIRS}")
MESSAGE("DEBUG opencv libs_dir: ${OpenCV_LIBS}")
MESSAGE("DEBUG opencv libraries: ${OpenCV_LIBRARIES}")




include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    )

ADD_LIBRARY(costmap_2d_test SHARED
    costmap_math.cpp
    array_parser.cpp
    observation_buffer.cpp
    footprint.cpp
    map_loader.cpp
    costmap_2d_test.cpp
)

target_link_libraries(costmap_2d_test
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


