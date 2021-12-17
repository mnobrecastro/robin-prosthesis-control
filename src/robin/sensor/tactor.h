#pragma once
#include "sensor1.h"

#include <array>

#define M_PI	3.14159265358979323846   // pi

namespace robin
{
	class Tactor :
		public Sensor1
	{
	public:
		Tactor();
		~Tactor();

		virtual void printInfo();

		virtual void runFeedback();

		void setSample(std::array<float,2> vals, size_t behaviour = 0);

		/*virtual void captureFrame() {}
		
		void setSample(float value);
		float getSample();

		void addChild(Sensor1* s);

		void feedChildren();
		virtual void fromParent(const float data);*/

	protected:

		std::array<float,2> data_ = {-1.0, 0.0};
		size_t behaviour_ = 0;
		std::array<float,2> getSample();

		/*float data_;
		std::mutex mu_data_;

		Sensor1* parent_ = nullptr;
		std::vector<Sensor1*> children_;

		//void feedChildren();
		//virtual void fromParent(const float data);
		*/

		std::thread thread_tactor_;

	private:
		static int counter_;
	};

}