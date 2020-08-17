#pragma once
#include "sensor.h"

#include <iostream>

namespace robin {

	class Sensor2 :
		public Sensor
	{
	public:
		Sensor2();
		~Sensor2();

	protected:

	private:
		static int counter;
	};

	//int Sensor::counter = 0;

}