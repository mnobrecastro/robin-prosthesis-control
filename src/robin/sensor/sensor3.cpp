#include "sensor3.h"

namespace robin {

	Sensor3::Sensor3()
	{ 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_clr_ = cloud_clr;
		std::cout << "A new Sensor3 was created" << std::endl;
	}
	Sensor3::~Sensor3() {}

	void Sensor3::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		pcl::copyPointCloud(cloud, *cloud_);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Sensor3::getPointCloud()
	{		
		mu_cloud_.lock();		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		mu_cloud_.unlock();
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Sensor3::getRawColored()
	{
		mu_cloud_.lock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_clr_));
		mu_cloud_.unlock();
		return cloud;
	}

	//

	void Sensor3::addChild(Sensor3* s)
	{
		children_.push_back(s);
	}

	void Sensor3::feedChildren()
	{
		mu_cloud_.lock();
		for (Sensor3* s : children_) {
			s->fromParent(*cloud_);
			s->fromParent(*cloud_clr_);
		}
		mu_cloud_.unlock();
	}

	void Sensor3::fromParent(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		mu_cloud_.lock();
		pcl::copyPointCloud(cloud, *cloud_);
		mu_cloud_.unlock();
	}

	void Sensor3::fromParent(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
	{
		mu_cloud_.lock();
		pcl::copyPointCloud(cloud, *cloud_clr_);
		mu_cloud_.unlock();
	}
}