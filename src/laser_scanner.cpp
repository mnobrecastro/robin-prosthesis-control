#include "laser_scanner.h"

namespace robin {

	LaserScanner::LaserScanner(CameraDepth* cam, float a, float b, float c, float d, float tol) :
		camera_(cam)
	{		
		cam->addChild(this);
		params_.a = a;
		params_.b = b;
		params_.c = c;
		params_.d = d;
		params_.tol = tol;
	}

	LaserScanner::LaserScanner(CameraDepth* cam, float a, float b, float tol)		
	{
		float theta(std::atan(a));
		float normal_x(std::cos(theta + M_PI / 2.0));
		float normal_y(std::sin(theta + M_PI / 2.0));
		LaserScanner(cam, normal_x, normal_y, 0.0, -normal_y*b, tol);
	}


	void LaserScanner::fromParent(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		mu_cloud_.lock();
		slicePointCloud(cloud, *cloud_);
		mu_cloud_.unlock();
	}

	void LaserScanner::slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_slice)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>());
		for (auto p : cloud.points) {
			if (std::abs(projectionDistance(p)) < params_.tol) {
				slice->push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
		pcl::copyPointCloud(*slice, cloud_slice);
	}

	float LaserScanner::projectionDistance(const pcl::PointXYZ& p)
	{
		return std::abs(params_.a * p.x + params_.b * p.y + params_.c * p.z + params_.d) / std::sqrt(std::pow(params_.a, 2) + std::pow(params_.b, 2) + std::pow(params_.c, 2)); 
	}

}