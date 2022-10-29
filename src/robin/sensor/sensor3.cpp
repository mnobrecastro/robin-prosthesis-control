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

#include "sensor3.h"

namespace robin {

	Sensor3::Sensor3()
		: cloud_(new pcl::PointCloud<pcl::PointXYZ>), cloud_clr_(new pcl::PointCloud<pcl::PointXYZRGB>)
	{ 
		std::cout << "A new Sensor3 was created" << std::endl;
	}

	Sensor3::~Sensor3() { }

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

	////

	void Sensor3::addChild(std::shared_ptr<Sensor3> s)
	{
		children_.push_back(s);
	}

	void Sensor3::feedChildren()
	{
		mu_cloud_.lock();
		for (auto s : children_) {
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