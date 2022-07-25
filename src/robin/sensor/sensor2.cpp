#include "sensor2.h"

namespace robin {

	Sensor2::Sensor2()
	{
		std::shared_ptr<cv::Mat> image(new cv::Mat);
		image_ = image;
		std::cout << "A new Sensor2 was created\n";
	}
	Sensor2::~Sensor2() {}

	void Sensor2::setImage(const cv::Mat& image)
	{
		*image_ = image.clone();
	}

	std::shared_ptr<cv::Mat> Sensor2::getImage()
	{
		mu_image_.lock();
		std::shared_ptr<cv::Mat> image(new cv::Mat(*image_));
		mu_image_.unlock();
		return image;
	}

	//

	void Sensor2::addChild(Sensor2* s)
	{
		children_.push_back(s);
	}

	void Sensor2::feedChildren()
	{
		mu_image_.lock();
		for (Sensor2* s : children_) {
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