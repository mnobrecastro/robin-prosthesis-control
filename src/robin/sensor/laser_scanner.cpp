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

#include "laser_scanner.h"

namespace robin {

	LaserScanner::LaserScanner(CameraDepth* cam, float a, float b, float c, float d, float tol) :
		camera_(cam)
	{		
		parent_ = cam;
		parent_->addChild(this);
		params_.a = a;
		params_.b = b;
		params_.c = c;
		params_.d = d;
		params_.tol = tol;
	}

	//LaserScanner::LaserScanner(CameraDepth* cam, float a, float b, float tol) :
	//	LaserScanner(cam, std::cos(std::atan(a) + M_PI / 2.0), std::sin(std::atan(a) + M_PI / 2.0), 0.0, -std::sin(std::atan(a) + M_PI / 2.0)*b, tol)
	//{}

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
			if (std::abs(this->projectionDistance(p)) < params_.tol/2) {
				slice->push_back(pcl::PointXYZ(p.x, p.y, p.z));
				//slice->push_back(this->projectIntoPlane(p));
			}
		}
		pcl::copyPointCloud(*slice, cloud_slice);
	}

	float LaserScanner::projectionDistance(const pcl::PointXYZ& p)
	{
		return (params_.a * p.x + params_.b * p.y + params_.c * p.z + params_.d) / std::sqrt(std::pow(params_.a, 2) + std::pow(params_.b, 2) + std::pow(params_.c, 2)); 
	}

	pcl::PointXYZ LaserScanner::projectIntoPlane(const pcl::PointXYZ& p)
	{
		pcl::PointXYZ proj_p;
		float d(projectionDistance(p));
		proj_p.x = p.x - d * params_.a / std::sqrt(std::pow(params_.a, 2) + std::pow(params_.b, 2) + std::pow(params_.c, 2));
		proj_p.y = p.y - d * params_.b / std::sqrt(std::pow(params_.a, 2) + std::pow(params_.b, 2) + std::pow(params_.c, 2));
		proj_p.z = p.z - d * params_.c / std::sqrt(std::pow(params_.a, 2) + std::pow(params_.b, 2) + std::pow(params_.c, 2));
		return proj_p;
	}

}