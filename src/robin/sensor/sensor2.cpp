#include "sensor2.h"

namespace robin {

	Sensor2::Sensor2()
		: image_(new cv::Mat)
	{
		std::cout << "A new Sensor2 was created\n";
	}

	Sensor2::~Sensor2() {}

	void Sensor2::setImage(const cv::Mat& image)
	{
		*image_ = image.clone();
	}

	std::shared_ptr<cv::Mat> Sensor2::getImage()
	{
		std::shared_ptr<cv::Mat> image(new cv::Mat);
		mu_image_.lock();		
		(*image_).copyTo(*image);
		mu_image_.unlock();
		return image;
	}

	////

	void Sensor2::addChild(std::shared_ptr<Sensor2> s)
	{
		children_.push_back(s);
	}

	void Sensor2::feedChildren()
	{
		mu_image_.lock();
		for (auto s : children_) {
			s->fromParent(*image_);
		}
		mu_image_.unlock();
	}

	void Sensor2::fromParent(const cv::Mat& image)
	{
		mu_image_.lock();
		*image_ = image.clone();
		mu_image_.unlock();
	}
}