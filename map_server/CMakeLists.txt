CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本
PROJECT( test ) #设定工程名
SET( CMAKE_CXX_COMPILER "g++") #设定编译器
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS -std=c++11)	
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
	
#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
	
#设定头文件目录
message(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR})
	
#增加子文件夹，也就是进入源代码文件夹继续构建
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV 2.4.8 REQUIRED )

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


file(GLOB_RECURSE SRCS
	"libsrc/*.cpp"
	"libsrc/*.c"
)
file(GLOB_RECURSE HDRS
	"libsrc/*.h"
)

ADD_EXECUTABLE(map_loader_test 
	src/map_loader_test.cpp 
	src/map_loader.cpp
	${SRCS}
	${HDRS}
)
TARGET_LINK_LIBRARIES(map_loader_test yaml-cpp SDL SDL_image ${OpenCV_LIBS})


