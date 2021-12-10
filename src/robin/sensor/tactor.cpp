#include "tactor.h"

namespace robin {

	Tactor::Tactor()
	{
		std::cout << "A new Tactor was created" << std::endl;
	}
	Tactor::~Tactor() {}

	void Tactor::printInfo() {}

	void Tactor::runFeedback() {}

	void Tactor::setSample(std::array<float,2> vals)
	{
		mu_data_.lock();
		data_[0] = vals[0];
		data_[1] = vals[1];
		mu_data_.unlock();
	}

	std::array<float,2> Tactor::getSample()
	{
		mu_data_.lock();
		std::array<float,2> data(data_);
		mu_data_.unlock();
		return data;
	}

	/*void Sensor1::setSample(const float value)
	{
		data_ = value;
	}

	float Sensor1::getSample()
	{
		float data;
		mu_data_.lock();
		data = data_;
		mu_data_.unlock();
		return data;
	}*/

	//

	/*void Sensor1::addChild(Sensor1* s)
	{
		children_.push_back(s);
	}

	void Sensor1::feedChildren()
	{
		float data;
		mu_data_.lock();
		data = data_;
		mu_data_.unlock();
		for (Sensor1* s : children_) {
			s->fromParent(data);
		}
	}

	void Sensor1::fromParent(const float value)
	{
		mu_data_.lock();
		data_ = value;
		mu_data_.unlock();
	}*/
}