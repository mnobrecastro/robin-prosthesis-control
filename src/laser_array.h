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
		LaserArraySingle(CameraDepth* cam);
		~LaserArraySingle();

	protected:

	};
	
	class LaserArrayCross :
		public LaserArray
	{
	public:
		LaserArrayCross();
		LaserArrayCross(CameraDepth* cam);
		~LaserArrayCross();

	protected:

	};

	class LaserArrayStar :
		public LaserArray
	{
	public:
		LaserArrayStar();
		LaserArrayStar(CameraDepth* cam);
		~LaserArrayStar();

	protected:
	};
}