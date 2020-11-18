#pragma once
#include "sensor_array.h"
#include "laser_scanner.h"
#include "camera_depth.h"

namespace robin {

	class LaserArray :
		public Sensor3Array
	{
	public:
		LaserArray() {}
		~LaserArray() {}

	private:
		/* Supressed method */
		using SensorArray::addSensor;
	};

	class LaserArraySingle :
		public LaserArray
	{
	public:
		LaserArraySingle();
		LaserArraySingle(CameraDepth* cam, float tol=0.001);
		~LaserArraySingle();

	protected:

	};
	
	class LaserArrayCross :
		public LaserArray
	{
	public:
		LaserArrayCross();
		LaserArrayCross(CameraDepth* cam, float tol=0.001);
		~LaserArrayCross();

	protected:

	};

	class LaserArrayStar :
		public LaserArray
	{
	public:
		LaserArrayStar();
		LaserArrayStar(CameraDepth* cam, float tol=0.001);
		~LaserArrayStar();

	protected:
	};
}