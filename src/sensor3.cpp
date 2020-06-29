#include "sensor3.h"

namespace robin {

	Sensor3::Sensor3() { std::cout << "A new Sensor was created" << std::endl; }
	Sensor3::~Sensor3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Sensor3::getPointCloud() const {
		return cloud_;
	}

}