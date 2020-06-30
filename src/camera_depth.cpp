#include "camera_depth.h"

#include <iostream>
#include <ctime>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/random_sample.h>

namespace robin {

	CameraDepth::CameraDepth() { std::cout << "A new CameraDepth was created" << std::endl; }
	
	CameraDepth::~CameraDepth() {}

	void CameraDepth::printInfo() {}

	void CameraDepth::captureFrame() { std::cout << "CameraDepth.captureFrame()" << std::endl; }

	void CameraDepth::setCrop(float x0=999.9, float x1=999.9, float y0=999.9, float y1=999.9, float z0=999.9, float z1=999.9)
	{
		/*for (auto i : ranges) {
			if (i[0] > i[1]) {
				return 1;
			}
		}*/
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };		
	}

	void CameraDepth::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr CameraDepth::trimPointCloud()/*char a = 'o', float width = 0.010)*/
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZ>());
		//if (a == 'o') {
			for (auto p : cloud_->points) {
				if (limits_[0] <= p.x && p.x <= limits_[1] &&
					limits_[2] <= p.y && p.y <= limits_[3] &&
					limits_[4] <= p.z && p.z <= limits_[5]) {
					cloud_trimmed->push_back(pcl::PointXYZ(p.x, p.y, p.z));
				}
			}
			/*} else if (a == '+') {
			for (auto p : cloud_.points) {
				if (//horizontal strip
					(ranges[0][0] <= p.x && p.x <= ranges[0][1] &&
					-width/2 <= p.y && p.y <= width/2 &&
					ranges[2][0] <= p.z && p.z <= ranges[2][1]) ||
					//vertical strip
					(-width/2 <= p.x && p.x <= width/2 &&
					ranges[1][0] <= p.y && p.y <= ranges[1][1] &&
					ranges[2][0] <= p.z && p.z <= ranges[2][1])) {
					cloud_trimmed.push_back(pcl::PointXYZ(p.x, p.y, p.z));
				}
			}
		} else {
			return 1;
		}*/
		return cloud_trimmed;
	}

	void CameraDepth::crop()
	{
		if (filterOnOff_) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Build a passthrough filter to remove unwated points
			pcl::PassThrough<pcl::PointXYZ> pass(true);
			if (false) {
				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("x");
				pass.setFilterLimits(limits_[0], limits_[1]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(limits_[2], limits_[3]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(limits_[4], limits_[5]);
				pass.filter(*cloud_);
			}
			else {
				cloud_ = trimPointCloud();
			}

			tf = std::time(0);
			std::cout << "PointCloud after filtering: " << cloud_->points.size() << " data points (in " << std::difftime(t0, tf) << " ms)." << std::endl;
		}
	}

	void CameraDepth::downsample()
	{
		if (downsampleOnOff_) {
			std::time_t t0,tf;
			t0 = std::time(0);

			// Downsampling the filtered point cloud
			if (true) {
				pcl::ApproximateVoxelGrid<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
				dsfilt.filter(*cloud_);
			}
			else {
				pcl::RandomSample<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setSample(2000);
				dsfilt.filter(*cloud_);
			}

			tf = std::time(0);
			std::cerr << "PointCloud after downsampling: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
					  << " data points (in " << std::difftime(t0, tf) << " ms)." << std::endl;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr CameraDepth::getFrame() { return cloud_; }

}