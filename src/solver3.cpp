#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
	}

	Solver3::~Solver3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPointCloud() const {
		return cloud_;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;
	}

	void Solver3::setUseNormals(bool seg_normals)
	{
		seg_normals_ = seg_normals;
	}

	void Solver3::setPlaneRemoval(bool seg_plane_removal)
	{
		seg_plane_removal_ = seg_plane_removal;
	}

	std::vector<robin::Sensor3*> Solver3::getSensors() const
	{
		return sensors_;
	}


	void Solver3::setCrop(float x0 = 999.9, float x1 = 999.9, float y0 = 999.9, float y1 = 999.9, float z0 = 999.9, float z1 = 999.9)
	{
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };
	}

	void Solver3::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::trimPointCloud()
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

	void Solver3::crop()
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

	void Solver3::downsample()
	{
		if (downsampleOnOff_) {
			std::time_t t0, tf;
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

	void Solver3::solve(robin::Primitive3& prim)
	{
		primitive_ = &prim;

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		for (Sensor3* s : sensors_) {
			*cloud_ += *s->getPointCloud();
		}

		this->crop();
		this->downsample();

		this->segment();
	}

	void Solver3::segment()
	{
		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform SAC segmentation." << std::endl;
			return;
		}
		
		// Check whether a Primitive3 has been provided
		if (primitive_ != nullptr) {
			// Perform initial plane removal
			if (seg_plane_removal_) {
				robin::Primitive3Plane plane;
				if (seg_obj_ptr_ != nullptr) {					
					plane.fit(cloud_, seg_obj_ptr_);
				}
				else{
					plane.fit(cloud_, seg_normals_);
				}
			}

			// Check whether an instance of Segmentation has been provided.
			if (seg_obj_ptr_ != nullptr) {
				std::cout << "Segmentation object has been provided!" << std::endl;
				primitive_->fit(cloud_, seg_obj_ptr_);				
			}
			else {
				std::cout << "NO segmentation object has been provided." << std::endl;
				primitive_->fit(cloud_, seg_normals_);				
			}
		}
	}

	void Solver3::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		/* Not implemented yet. */
	}
}