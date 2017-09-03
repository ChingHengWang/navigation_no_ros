#include <costmap_2d_test/map_loader.h>

MapLoader::MapLoader(){
    }

MapLoader::MapLoader(std::string file_path){
    fname = file_path;
    }
//MapLoader::~MapLoader(){}
void MapLoader::load(arc::OccupancyGrid& map)
{

   
  std::ifstream fin(fname.c_str());
  if (fin.fail()) {
    printf("Map_server could not open %s.", fname.c_str());
    exit(-1);
  }
  printf("successful open yaml file\n");
  YAML::Node doc = YAML::Load(fin);
  doc["resolution"] >> res;
  doc["negate"] >> negate;
  doc["occupied_thresh"] >> occ_th;
  doc["free_thresh"] >> free_th;
  doc["origin"][0] >> origin[0];
  doc["origin"][1] >> origin[1];
  doc["origin"][2] >> origin[2];
  doc["image"] >> mapfname;
  printf("successful open yaml file2\n");

  SDL_Surface* map_img;
  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  unsigned int i,j;
  int rowstride, n_channels, avg_channels;
  int color_sum;
  char* fname_copy = strdup(fname.c_str());
  mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
  free(fname_copy);
  map_img=IMG_Load(mapfname.c_str());
  printf("image load finish\n");

  printf("image load finish0000\n");
  printf("map img w %d\n",map_img->w);

  map.info.width = map_img->w;
  map.info.height = map_img->h;
  printf("image load finish0\n");


  map.info.resolution = res;
  map.info.origin.position.x = *(origin);
  map.info.origin.position.y = *(origin+1);
  map.info.origin.position.z = 0.0;  
  printf("image load finish1\n");

  slam_tf::Quaternion q;
  q.setRPY(0,0, *(origin+2));
  map.info.origin.orientation.x = q.x();
  map.info.origin.orientation.y = q.y();
  map.info.origin.orientation.z = q.z();
  map.info.origin.orientation.w = q.w();
  printf("image load finish2\n");

  map.data.resize(map.info.width*map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = map_img->pitch;
  n_channels = map_img->format->BytesPerPixel;
  //Because TRINARY so anv_channels = n_channels
  avg_channels=n_channels;
  printf("image load finish3\n");

  pixels = (unsigned char*)(map_img->pixels);

  for(j = 0; j < map.info.height; j++)
  {
    for(i =0; i < map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + (map.info.height-j)*rowstride + i*n_channels;
      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      occ = (255 - *p) / 255.0;
      
      if(occ > occ_th){
        value = +100;
  	    //printf("map is 100\n");
      }
      else if(occ < free_th){
        value = 0;
        //printf("map is 0\n");
      }
      else {
        value = -1;
  	    //printf("map is -1\n");
      }
      //map.data[MAP_IDX(map.width,i,map.height - j - 1)] = value;
      map.data[MAP_IDX(map.info.width,i,j)] = value;
    }
  }

}//load 

