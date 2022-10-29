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

#include "sensor.h"

namespace robin {

	std::string Sensor::getID() const {
		return id_;
	}

	void Sensor::setID(std::string str) {
		id_ = str;
	}

	void Sensor::addChild(std::shared_ptr<Sensor> s)
	{
		children_.push_back(s);
	}
}