#pragma once
#include "robin/sensor/sensor.h"
#include "robin/sensor/sensor_array.h"
#include "robin/primitive/primitive.h"

#include <vector>

namespace robin
{
	class Solver
	{
	public:
		Solver() {}
		//Solver(robin::Primitive*);
		~Solver() {}

		virtual void addSensor(robin::Sensor*) {}

		virtual void addSensor(robin::SensorArray*) {}

		virtual void setPrimitive(robin::Primitive*) {}

	protected:
		std::vector<robin::Sensor*> sensors_;
		robin::Primitive* primitive_ = nullptr;
	};
}