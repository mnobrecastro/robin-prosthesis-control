#include "laser_array.h"

namespace robin
{


	/// LASER_ARRAY_CROSS

	LaserArrayCross::LaserArrayCross() {}

	LaserArrayCross::LaserArrayCross(CameraDepth* cam)
	{
		LaserScanner* laser_h(new LaserScanner(cam, 0.0, 1.0, 0.0, 0.0, 0.001)); // 0 deg
		sensors_.push_back(laser_h);
		LaserScanner* laser_v(new LaserScanner(cam, 1.0, 0.0, 0.0, 0.0, 0.001)); // 90 deg
		sensors_.push_back(laser_v);
	}

	LaserArrayCross::~LaserArrayCross() {}



	/// LASER_ARRAY_STAR

	LaserArrayStar::LaserArrayStar() {}

	LaserArrayStar::LaserArrayStar(CameraDepth* cam)
	{
		LaserScanner* laser_0(new LaserScanner(cam, 0.0, 1.0, 0.0, 0.0, 0.001)); // 0 deg
		sensors_.push_back(laser_0);
		LaserScanner* laser_1(new LaserScanner(cam, -1.0, 1.0, 0.0, 0.0, 0.001)); // 45deg
		sensors_.push_back(laser_1);
		LaserScanner* laser_2(new LaserScanner(cam, 1.0, 0.0, 0.0, 0.0, 0.001)); // 90 deg
		sensors_.push_back(laser_2);
		LaserScanner* laser_3(new LaserScanner(cam, 1.0, 1.0, 0.0, 0.0, 0.001)); // 135 deg
		sensors_.push_back(laser_3);
	}

	LaserArrayStar::~LaserArrayStar() {}
}