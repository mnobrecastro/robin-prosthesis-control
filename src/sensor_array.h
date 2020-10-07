#pragma once
#include "sensor.h"
#include "sensor1.h"
#include "sensor2.h"
#include "sensor3.h"
#include "sensor4.h"

#include <vector>

namespace robin {

	class SensorArray
	{
	public:
		std::vector<Sensor*> getSensors() const;
		virtual void addSensor(Sensor* s);

	protected:
		/* Prevents this class from being initialized. */
		SensorArray() {}
		~SensorArray() {}

		std::vector<Sensor*> sensors_;
	};

	class Sensor1Array :
		public SensorArray
	{
	public:
		Sensor1Array() {}
		~Sensor1Array() {}

		std::vector<Sensor1*> getSensors() const;
		void addSensor(Sensor1* s);

	protected:
		std::vector<Sensor1*> sensors_;
	};

	class Sensor2Array :
		public SensorArray
	{
	public:
		Sensor2Array() {}
		~Sensor2Array() {}

		std::vector<Sensor2*> getSensors() const;
		void addSensor(Sensor2* s);

	protected:
		std::vector<Sensor2*> sensors_;
	};

	class Sensor3Array :
		public SensorArray
	{
	public:
		Sensor3Array() {}
		~Sensor3Array() {}

		std::vector<Sensor3*> getSensors() const;
		void addSensor(Sensor3* s);

	protected:
		std::vector<Sensor3*> sensors_;
	};

	class Sensor4Array :
		public SensorArray
	{
	public:
		Sensor4Array() {}
		~Sensor4Array() {}

		std::vector<Sensor4*> getSensors() const;
		void addSensor(Sensor4* s);

	protected:
		std::vector<Sensor4*> sensors_;
	};
}