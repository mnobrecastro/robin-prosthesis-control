#include "primitive3_ellipse.h"

namespace robin
{
	Primitive3Ellipse::Primitive3Ellipse()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 7-element Ellipse3d coefficients
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
		coefficients_->values.push_back(0.0); //4
	}
	Primitive3Ellipse::~Primitive3Ellipse() {}
	
	void Primitive3Ellipse::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		//viewer->addCylinder(*coefficients_, "Ellipse"); // Renders just like a cylinder
	}

	void Primitive3Ellipse::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.000;
		coefficients_->values[4] = 0.000;
	}


	void Primitive3Ellipse::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_ELLIPSE2D);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_ELLIPSE2D);
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Ellipse::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_ELLIPSE2D);
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
	bool Primitive3Ellipse::is_fit_valid()
	{
		if (!cloud_->points.empty()) {
			/* x_min, x_max, y_min, y_max, z_min, z_max. */
			std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
			/* Checks the z-coordinate of the Primitive3Ellipse center. */
			if (coefficients_->values[2] > ranges[4]) {
				return true;
			}
		}
		this->reset();
		return false;
	}

	void Primitive3Ellipse::correct_coefficients()
	{
		/** (sac_model_Ellipse3d.h) SampleConsensusModelEllipse3D defines a model for 3D Ellipse segmentation.
		* The model coefficients are defined as:
		*   - \b f1.x : the X coordinate of the ellipse's focal point f1
		*   - \b f1.y : the Y coordinate of the ellipse's focal point f1
		*   - \b f2.x : the X coordinate of the ellipse's focal point f2
		*   - \b f2.y : the Y coordinate of the ellipse's focal point f2
		*   - \b length : the ellipse's lenght of the sum of distances to foci
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
	void Primitive3Ellipse::update_properties()
	{
		//properties_.f1_x = coefficients_->values[0];
		//properties_.f1_y = coefficients_->values[1];
		//properties_.f2_x = coefficients_->values[2];
		//properties_.f2_y = coefficients_->values[3];
		properties_.radius = coefficients_->values[4];
	}
}