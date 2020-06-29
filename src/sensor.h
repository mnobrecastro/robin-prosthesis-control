#pragma once

#include <iostream>
#include <string>

namespace robin {

	class Sensor
	{
	public:
		Sensor() {}
		~Sensor() {}

		std::string getID() const;
		void setID(std::string);

		virtual void printInfo() {}

		virtual void start(bool) { std::cout << "Wrong one!" << std::endl; }
		virtual void captureFrame() {}

		//virtual auto getFrame() {};

		virtual void crop() {}
		virtual void downsample() {}

	protected:
		std::string id_;
	};

}