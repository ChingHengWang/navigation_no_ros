#include <costmap_2d_test/header.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <string>
#include <vector>
#include <algorithm>

#include <yaml-cpp/yaml.h>
#include <SDL/SDL_image.h>
using namespace arc;
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace slam_tf
{
    class Quaternion
    {
    public:
      Quaternion(){
      }


      Quaternion(const double& x, const double& y, const double& z){
        m_floats[0] = x, m_floats[1] = y, m_floats[2] = z;
      }


      Quaternion(const double& x, const double& y, const double& z, const double& w){
        m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
      }
      void setRPY(const double& roll, const double& pitch, const double& yaw)
        {
        double halfYaw = double(yaw) * double(0.5);  
        double halfPitch = double(pitch) * double(0.5);  
        double halfRoll = double(roll) * double(0.5);  
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        double cosPitch = cos(halfPitch);
        double sinPitch = sin(halfPitch);
        double cosRoll = cos(halfRoll);
        double sinRoll = sin(halfRoll);
        setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                     cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                     cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                     cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
        }
        const double& x() const { return m_floats[0]; }
        const double& y() const { return m_floats[1]; }
        const double& z() const { return m_floats[2]; }
        const double& w() const { return m_floats[3]; }
    protected:
      void setValue(const double& x, const double& y, const double& z,const double& w)
      {
        m_floats[0]=x;
        m_floats[1]=y;
        m_floats[2]=z;
        m_floats[3]=w;
      }

      private:
        double m_floats[4];

    };
}

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}


class MapLoader
{
public:
  MapLoader();
  MapLoader(std::string file_path);

  void load(arc::OccupancyGrid& map);

private:
  std::string fname;
  std::string mapfname;   
  double res;
  double origin[3];
  int negate;
  double occ_th, free_th;
  double occ;
};


