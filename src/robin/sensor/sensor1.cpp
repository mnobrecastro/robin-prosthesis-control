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