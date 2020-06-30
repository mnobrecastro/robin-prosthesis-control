#include "primitive3_sphere.h"

namespace robin
{
	void Primitive3Sphere::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addSphere(*coefficients_, "cylinder");
	}


	void Primitive3Sphere::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_SPHERE);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_SPHERE);
		}

		/* 1. Validate the fit. */
		if (!this->is_fit_valid()) {
			coefficients_->values[0] = -0.001;
			coefficients_->values[1] = -0.001;
			coefficients_->values[2] = -0.001;
			coefficients_->values[3] = 0.001;
		}
		else {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Sphere::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_SPHERE);
		if (typeid(seg) == typeid(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*)) {
			fit_sample_consensus_with_normals(cloud, seg);
		}
		else {
			fit_sample_consensus(cloud, seg);
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	/* Checks if the fit is valid. */
	bool Primitive3Sphere::is_fit_valid()
	{
		if (!cloud_->points.empty()) {
			/* x_min, x_max, y_min, y_max, z_min, z_max. */
			std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
			/* Checks the z-coordinate of the Primitive3Sphere center. */
			if (coefficients_->values[2] > ranges[4]) {
				return true;
			}
		}
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.001;
		return false;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Sphere::correct_coefficients()
	{
		/* Nothing to correct. */
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Sphere::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.radius = coefficients_->values[3];
	}
}