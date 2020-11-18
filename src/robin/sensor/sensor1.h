#pragma once
#include "sensor.h"

#include <iostream>
#include <vector>
#include <mutex>

namespace robin
{
	class Sensor1 :
		public Sensor
	{
	public:
		Sensor1();
		~Sensor1();

		void setSample(float value);
		float getSample();

		void addChild(Sensor1* s);

		void feedChildren();
		virtual void fromParent(const float data);

	protected:
		float data_;
		std::mutex mu_data_;

		Sensor1* parent_ = nullptr;
		std::vector<Sensor1*> children_;

		//void feedChildren();
		//virtual void fromParent(const float data);

	private:
		static int counter_;
	};

}