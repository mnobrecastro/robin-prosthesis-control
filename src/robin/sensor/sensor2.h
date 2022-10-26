#pragma once
#include "sensor.h"

#include <iostream>
//#include <memory>
#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>

namespace robin {

	class Sensor2 :
		public Sensor
	{
	public:
		Sensor2();
		~Sensor2();

		void setImage(const cv::Mat& image);
		std::shared_ptr<cv::Mat> getImage();

		void addChild(Sensor2* s);

	protected:
		std::unique_ptr<cv::Mat> image_;
		std::mutex mu_image_;

		Sensor2* parent_ = nullptr;
		std::vector<Sensor2*> children_;

		void feedChildren();
		virtual void fromParent(const cv::Mat& cloud);

	private:
		static int counter;
	};

	//int Sensor::counter = 0;

}