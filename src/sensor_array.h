#pragma once
#include "sensor.h"

#include <vector>

namespace robin {

	class SensorArray :
		public Sensor
	{
	public:
		~SensorArray();

		virtual void addSensor(Sensor* s);

	protected:
		std::vector<Sensor*> sensors_;

	};
}