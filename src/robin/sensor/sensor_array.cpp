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

	std::vector<Sensor1*> Sensor1Array::getSensors() const
	{
		return sensors_;
	}

	void Sensor1Array::addSensor(Sensor1* s)
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