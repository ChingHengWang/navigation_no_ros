#include <map_server/map_loader.h>

#include <debug/cv_debug_header.h>
#include "../libsrc/inih/cpp/INIReader.h"
string ini_path = "../load_param.ini";
INIReader reader(ini_path);
const std::string map_path = reader.Get("map_loader","map_path","UNKNOW");



int main(){

  MapLoader ml(map_path.c_str());
  OccupancyGrid output;
  ml.load(output);    

//////////////////////////////////////////////////////
  cv::Mat M=cv::Mat(output.info.height,output.info.width,CV_8UC1);
  memcpy(M.data,output.data.data(),output.data.size()*sizeof(int8_t));

  cv::Mat M_cv=cv::Mat(output.info.height,output.info.width,CV_8UC1);

  createOpenCVDebugMat(M,M_cv);

  imshow("M_cv_output", M_cv);
  cvWaitKey();
/////////////////////////////////////////////////////

  return 0; 
}

