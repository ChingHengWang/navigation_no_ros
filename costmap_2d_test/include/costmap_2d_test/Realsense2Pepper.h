#ifndef __REALSENS_TO_PEPPER_H__
#define __REALSENS_TO_PEPPER_H__

#include "IntelDevice.h"
//----------------------------------------------------------------------------------------------------
namespace Pepper_XIAOPENG
{
	struct Header
	{
		unsigned int seq;
		time_t       stamp;
		std::string  frame_id;
	};

	struct Point
	{
		float x;
		float y;
		float z;

		Point(){}
		Point(float _x, float _y, float _z)
			:x(_x), y(_y), z(_z)
		{}
	};

	struct ChannelFloat32
	{
		std::string         name;
		std::vector <float> values;
	};

	class PointCloud
	{
	public:
		Header                     header;
		std::vector<Point>         points;
		std::vector<ChannelFloat32> channels;

		//! 
		void TransormPoints(float R[9], float T[3])
		{
			for (auto i = 0; i < points.size(); i++ )
			{
				float x = points[i].x;
				float y = points[i].y;
				float z = points[i].z;

				points[i].x = R[0] * x + R[1] * y + R[2] * z + T[0];
				points[i].y = R[3] * x + R[4] * y + R[5] * z + T[1];
				points[i].z = R[6] * x + R[7] * y + R[8] * z + T[2];
			}
		}
		// R<1,0,0; 0,1,0; 0,0,1>  T<0,0,1>
		void TransormPoints(double R[9], double T[3])
		{
			float _R[3], _T[3];
			for (int i = 0; i < 3; i++)	_T[i] = (float)T[i];
			for (int i = 0; i < 9; i++)	_R[i] = (float)R[i];
			TransormPoints(_R, _T);
		}
	};
};
//------------------------------------------------------------
//! Convert intel realsense frame to pepper:::cloud
void ConvertFrame2PiontCloud(rs::device& dev, Pepper_XIAOPENG::PointCloud& pc, float fMinRange = 0.05f, float fMaxRange = 6.0f)
{
	float fMin = fMinRange; //  0.05f; // m
	float fMax = fMaxRange; // 6.0f;

	const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
	auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
	for (int y = 0; y < depth_intrin.height; ++y)
	{
		for (int x = 0; x < depth_intrin.width; ++x)
		{
			if (points->z > fMin && points->z < fMax)
			{
				float x = points->x;
				float y = points->y;
				float z = points->z;
				//converto pepper axis frame
				pc.points.push_back(Pepper_XIAOPENG::Point( z, x, -y));
			}
			++points;
		}
	}
}
//----------------------------------------------------------------------------------------------------
#endif