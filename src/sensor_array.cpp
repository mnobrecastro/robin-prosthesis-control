#include "sensor_array.h"

namespace robin
{
	SensorArray::~SensorArray() {}

	void SensorArray::addSensor(Sensor* s)
	{
		sensors_.push_back(s);
	}
}