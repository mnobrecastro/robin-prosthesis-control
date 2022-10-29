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

#include "camera_rgb.h"

namespace robin {

	CameraRgb::CameraRgb() { std::cout << "A new CameraRgb was created.\n"; }
	
	CameraRgb::~CameraRgb() {}

	void CameraRgb::printInfo() {}

	void CameraRgb::captureFrame() { std::cout << "CameraRgb.captureFrame().\n"; }

	void CameraRgb::setCrop(float x0=999.9, float x1=999.9, float y0=999.9, float y1=999.9, float z0=999.9, float z1=999.9)
	{
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };		
	}

	void CameraRgb::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
	}
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr CameraRgb::trimPointCloud()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZ>());
		for (auto p : cloud_->points) {
			if (limits_[0] <= p.x && p.x <= limits_[1] &&
				limits_[2] <= p.y && p.y <= limits_[3] &&
				limits_[4] <= p.z && p.z <= limits_[5]) {
				cloud_trimmed->push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
		return cloud_trimmed;
	}

	void CameraRgb::crop()
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
			std::cout << "PointCloud after filtering: " << cloud_->points.size() << " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
	}

	void CameraRgb::downsample()
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
					  << " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
	}
	*/
}