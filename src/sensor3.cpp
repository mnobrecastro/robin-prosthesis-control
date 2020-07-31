#include "sensor3.h"

namespace robin {

	Sensor3::Sensor3()
	{ 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		std::cout << "A new Sensor3 was created" << std::endl;
	}
	Sensor3::~Sensor3() {}

	void Sensor3::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		if (false) {
			pcl::copyPointCloud(cloud, *cloud_);
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Sensor3::getPointCloud()
	{		
		mu_cloud_.lock();		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		mu_cloud_.unlock();
		return cloud;
	}

	void Sensor3::addChild(Sensor3* s)
	{
		children_.push_back(s);
	}

	void Sensor3::feedChildren()
	{
		mu_cloud_.lock();
		for (Sensor3* s : children_) {
			//if (this == s->parent_) {
			s->fromParent(*cloud_);
			//}
		}
		mu_cloud_.unlock();
	}

	void Sensor3::fromParent(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		mu_cloud_.lock();
		pcl::copyPointCloud(cloud, *cloud_);
		mu_cloud_.unlock();
	}
}