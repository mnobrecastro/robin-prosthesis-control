#include "sensor.h"

namespace robin {

	std::string Sensor::getID() const {
		return id_;
	}

	void Sensor::setID(std::string str) {
		id_ = str;
	}
}