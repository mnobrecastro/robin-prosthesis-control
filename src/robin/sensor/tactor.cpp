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

#include "tactor.h"

namespace robin {

	Tactor::Tactor()
	{
		std::cout << "A new Tactor was created" << std::endl;
	}
	Tactor::~Tactor() {}

	void Tactor::printInfo() {}

	void Tactor::runFeedback() {}

	void Tactor::setSample(std::array<float,2> vals, size_t behaviour)
	{
		mu_data_.lock();
		data_[0] = vals[0];
		data_[1] = vals[1];
		behaviour_ = behaviour;
		mu_data_.unlock();
	}

	std::array<float,2> Tactor::getSample()
	{
		mu_data_.lock();
		std::array<float,2> data(data_);
		mu_data_.unlock();
		return data;
	}

}