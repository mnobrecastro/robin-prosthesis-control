#include "sensor_array.h"

namespace robin
{
	std::vector<Sensor*> SensorArray::getSensors() const
	{
		return sensors_;
	}

	void SensorArray::addSensor(Sensor* s)
	{
		sensors_.push_back(s);
	}

	//

	std::vector<Sensor2*> Sensor2Array::getSensors() const
	{
		return sensors_;
	}

	void Sensor2Array::addSensor(Sensor2* s)
	{
		sensors_.push_back(s);
	}

	//

	std::vector<Sensor3*> Sensor3Array::getSensors() const
	{
		return sensors_;
	}

	void Sensor3Array::addSensor(Sensor3* s)
	{
		sensors_.push_back(s);
	}

	//

	std::vector<Sensor4*> Sensor4Array::getSensors() const
	{
		return sensors_;
	}

	void Sensor4Array::addSensor(Sensor4* s)
	{
		sensors_.push_back(s);
	}
}