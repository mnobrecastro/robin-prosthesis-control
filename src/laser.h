#pragma once
#include "sensor3.h"
#include "camera_depth.h"

namespace robin {

	class Laser :
		public Sensor3
	{
	public:
		Laser() {}
		Laser(CameraDepth& cam) {}

	protected:

	};
}

