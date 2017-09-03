#map_server
* version 0.0.2
## map_loader_test.cpp
test function
## map_loader.cpp
map_loader

## map folder
* navigation_no_ros/pkg/map_server/map


## make and build and run

		mkdir build && cd build

		cmake ..

		make

		cd ../bin

		./map_loader_test

## INI Parser
* it's a parser can parse ini config file

* ini file is load_param.ini
  * first part : [map_loader]
  * second part : map_path

* CMakeList.txt need to add

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

* code add these lines

		#include "../libsrc/inih/cpp/INIReader.h"
		string ini_path = "../load_param.ini";
		INIReader reader(ini_path);
		const std::string map_path = reader.Get("map_loader","map_path","UNKNOW");

* so taht MapLoader can use this path from ini file

		MapLoader ml(map_path.c_str());
 
