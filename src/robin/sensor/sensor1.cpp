#include "sensor1.h"

namespace robin {
	Sensor1::Sensor1()
		: data_(0.0)
	{
		std::cout << "A new Sensor1 was created" << std::endl;
	}

	Sensor1::~Sensor1() { }

	void Sensor1::setSample(const float value)
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
	}

	//

	void Sensor1::addChild(std::shared_ptr<Sensor1> s)
	{
		children_.push_back(s);
	}

	void Sensor1::feedChildren()
	{
		float data;
		mu_data_.lock();
		data = data_;
		mu_data_.unlock();
		for (auto s : children_) {
			s->fromParent(data);
		}
	}

	void Sensor1::fromParent(const float value)
	{
		mu_data_.lock();
		data_ = value;
		mu_data_.unlock();
	}
}