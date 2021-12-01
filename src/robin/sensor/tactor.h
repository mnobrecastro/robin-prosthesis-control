#pragma once
#include "sensor1.h"

//#include <iostream>
//#include <vector>
//#include <mutex>

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

		/*virtual void captureFrame() {}
		
		void setSample(float value);
		float getSample();

		void addChild(Sensor1* s);

		void feedChildren();
		virtual void fromParent(const float data);*/

	protected:
		/*float data_;
		std::mutex mu_data_;

		Sensor1* parent_ = nullptr;
		std::vector<Sensor1*> children_;

		//void feedChildren();
		//virtual void fromParent(const float data);
		*/

		std::thread thread_feedback_;

	private:
		static int counter_;
	};

}