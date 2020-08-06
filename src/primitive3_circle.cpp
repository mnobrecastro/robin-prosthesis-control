#include "primitive3_circle.h"

namespace robin
{
	Primitive3Circle::Primitive3Circle()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 7-element circle3d coefficients
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
		coefficients_->values.push_back(0.0); //4
		coefficients_->values.push_back(0.0); //5
		coefficients_->values.push_back(0.0); //6
	}
	Primitive3Circle::~Primitive3Circle() {}
	
	void Primitive3Circle::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addCylinder(*coefficients_, "circle"); // Renders just like a cylinder
	}

	void Primitive3Circle::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.000;
		coefficients_->values[4] = 0.000;
		coefficients_->values[5] = 0.000;
		coefficients_->values[6] = 0.001;
	}


	void Primitive3Circle::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CIRCLE3D);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CIRCLE3D);
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Circle::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_CIRCLE3D);
		if (typeid(*seg) == typeid(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>)) {
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
	bool Primitive3Circle::is_fit_valid()
	{
		if (!cloud_->points.empty()) {
			/* x_min, x_max, y_min, y_max, z_min, z_max. */
			std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
			/* Checks the z-coordinate of the Primitive3Sphere center. */
			if (coefficients_->values[2] > ranges[4]) {
				return true;
			}
		}
		this->reset();
		return false;
	}

	void Primitive3Circle::correct_coefficients()
	{
		/** (sac_model_circle3d.h) SampleConsensusModelCircle3D defines a model for 3D circle segmentation.
		* The model coefficients are defined as:
		*   - \b center.x : the X coordinate of the circle's center
		*   - \b center.y : the Y coordinate of the circle's center
		*   - \b center.z : the Z coordinate of the circle's center
		*   - \b radius   : the circle's radius
		*   - \b normal.x : the X coordinate of the normal's direction
		*   - \b normal.y : the Y coordinate of the normal's direction
		*   - \b normal.z : the Z coordinate of the normal's direction
		*/
		Eigen::Vector3f normal(coefficients_->values[4], coefficients_->values[5], coefficients_->values[6]);

		/** (sac_model_cylinder.h) SampleConsensusModelCylinder defines a model for 3D cylinder segmentation.
		* The model coefficients are defined as:
		*   - \b point_on_axis.x  : the X coordinate of a point located on the cylinder axis
		*   - \b point_on_axis.y  : the Y coordinate of a point located on the cylinder axis
		*   - \b point_on_axis.z  : the Z coordinate of a point located on the cylinder axis
		*   - \b axis_direction.x : the X coordinate of the cylinder's axis direction
		*   - \b axis_direction.y : the Y coordinate of the cylinder's axis direction
		*   - \b axis_direction.z : the Z coordinate of the cylinder's axis direction
		*   - \b radius           : the cylinder's radius
		*/	
		coefficients_->values[6] = coefficients_->values[3];
		coefficients_->values[3] = normal[0] * 0.001/normal.norm();
		coefficients_->values[4] = normal[1] * 0.001/normal.norm();
		coefficients_->values[5] = normal[2] * 0.001/normal.norm();
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Circle::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.axis_x = coefficients_->values[3];
		properties_.axis_y = coefficients_->values[4];
		properties_.axis_z = coefficients_->values[5];
		properties_.radius = coefficients_->values[6];
	}
}