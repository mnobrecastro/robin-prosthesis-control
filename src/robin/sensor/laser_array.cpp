/*
 * Semi-autonomous Prosthesis Control Using Computer Vision - Robin C++ framework
 *
 * Author: Miguel Nobre Castro (mnobrecastro@gmail.com)
 *
 *
 * This work was performed at the Department of Health Science and Technology, Aalborg
 * University, under the supervision of Professor Strahinja Dosen (sdosen@hst.aau.dk),
 * and was supported by the Independent Research Fund Denmark through the project ROBIN
 * "RObust Bidirectional human-machine INterface for natural control and feedback in
 * hand prostheses" (8022-00243A).
 */

#include "laser_array.h"

namespace robin
{
	/// LASER_ARRAY_SINGLE

	LaserArraySingle::LaserArraySingle() {}

	LaserArraySingle::LaserArraySingle(CameraDepth* cam, float tol)
	{
		LaserScanner* laser_h(new LaserScanner(cam, 0.0, 1.0, 0.0, 0.0, tol)); // 0 deg
		sensors_.push_back(laser_h);
	}

	LaserArraySingle::~LaserArraySingle() {}



	/// LASER_ARRAY_CROSS

	LaserArrayCross::LaserArrayCross() {}

	LaserArrayCross::LaserArrayCross(CameraDepth* cam, float tol)
	{
		LaserScanner* laser_h(new LaserScanner(cam, 0.0, 1.0, 0.0, 0.0, tol)); // 0 deg
		sensors_.push_back(laser_h);
		LaserScanner* laser_v(new LaserScanner(cam, 1.0, 0.0, 0.0, 0.0, tol)); // 90 deg
		sensors_.push_back(laser_v);
	}

	LaserArrayCross::~LaserArrayCross() {}



	/// LASER_ARRAY_STAR

	LaserArrayStar::LaserArrayStar() {}

	LaserArrayStar::LaserArrayStar(CameraDepth* cam, float tol)
	{
		LaserScanner* laser_0(new LaserScanner(cam, 0.0, 1.0, 0.0, 0.0, tol)); // 0 deg
		sensors_.push_back(laser_0);
		LaserScanner* laser_1(new LaserScanner(cam, -1.0, 1.0, 0.0, 0.0, tol)); // 45deg
		sensors_.push_back(laser_1);
		LaserScanner* laser_2(new LaserScanner(cam, 1.0, 0.0, 0.0, 0.0, tol)); // 90 deg
		sensors_.push_back(laser_2);
		LaserScanner* laser_3(new LaserScanner(cam, 1.0, 1.0, 0.0, 0.0, tol)); // 135 deg
		sensors_.push_back(laser_3);
	}

	LaserArrayStar::~LaserArrayStar() {}
}