#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <thread>

namespace robin {

	class Sensor
	{
	public:
		Sensor() {}
		~Sensor() {}

		std::string getID() const;
		void setID(std::string);

		virtual void printInfo() {}

		virtual void addChild(std::shared_ptr<Sensor> s);

		//virtual auto getFrame() {};

	protected:
		std::string id_;

		std::shared_ptr<Sensor> parent_ = nullptr;
		std::vector<std::shared_ptr<Sensor>> children_;

		virtual void feedChildren() {}

		virtual void start(bool) { std::cout << "Wrong one!" << std::endl; }
		virtual void captureFrame() {}
		std::thread thread_capture_;

	private:

	};

}