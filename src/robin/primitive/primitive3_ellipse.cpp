#include "primitive3_ellipse.h"

namespace robin
{
	Primitive3Ellipse::Primitive3Ellipse()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 11-element Ellipse3d coefficients
		coefficients_->values.push_back(0.0); //0 par_cx
		coefficients_->values.push_back(0.0); //1 par_cy
		coefficients_->values.push_back(0.0); //2 par_cz
		coefficients_->values.push_back(0.0); //3 par_a
		coefficients_->values.push_back(0.0); //4 par_b
		coefficients_->values.push_back(0.0); //5 par_nx
		coefficients_->values.push_back(0.0); //6 par_ny
		coefficients_->values.push_back(0.0); //7 par_nz
		coefficients_->values.push_back(0.0); //8 par_tx
		coefficients_->values.push_back(0.0); //9 par_ty
		coefficients_->values.push_back(0.0); //10 par_tz
	}
	Primitive3Ellipse::~Primitive3Ellipse() {}
	
	void Primitive3Ellipse::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		// Renders the 3D ellipse using 100 points
		pcl::PointXYZ center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ point_x(properties_.center_x + properties_.e0_x, properties_.center_y + properties_.e0_y, properties_.center_z + properties_.e0_z);
		pcl::PointXYZ point_y(properties_.center_x + properties_.e1_x, properties_.center_y + properties_.e1_y, properties_.center_z + properties_.e1_z);
		pcl::PointXYZ point_z(properties_.center_x + properties_.axis_x, properties_.center_y + properties_.axis_y, properties_.center_z + properties_.axis_z);
		viewer->addLine<pcl::PointXYZ>(center, point_x, 1.0, 0.4, 0.4, "axis0");
		viewer->addLine<pcl::PointXYZ>(center, point_y, 0.4, 1.0, 0.4, "axis1");
		viewer->addLine<pcl::PointXYZ>(center, point_z, 0.4, 0.4, 1.0, "axis2");
	}

	void Primitive3Ellipse::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.000;
		coefficients_->values[4] = 0.000;
		coefficients_->values[5] = 0.000;
		coefficients_->values[6] = 0.000;
		coefficients_->values[7] = 0.000;
		coefficients_->values[8] = 0.000;
		coefficients_->values[9] = 0.000;
		coefficients_->values[10] = 0.000;
	}


	void Primitive3Ellipse::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_ELLIPSE3D);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_ELLIPSE3D);
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
		seg->setModelType(pcl::SACMODEL_ELLIPSE3D);
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
		/** (sac_model_ellipse3d.h) SampleConsensusModelEllipse3D defines a model for 3D Ellipse segmentation.
		* The model coefficients are defined as:
		*   - \b f1.x : the X coordinate of the ellipse's focal point f1
		*   - \b f1.y : the Y coordinate of the ellipse's focal point f1
		*   - \b f2.x : the X coordinate of the ellipse's focal point f2
		*   - \b f2.y : the Y coordinate of the ellipse's focal point f2
		*   - \b length : the ellipse's lenght of the sum of distances to foci
		*/
		//Eigen::Vector3f normal(coefficients_->values[4], coefficients_->values[5], coefficients_->values[6]);

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
		//coefficients_->values[6] = coefficients_->values[3];
		//coefficients_->values[3] = normal[0] * 0.001/normal.norm();
		//coefficients_->values[4] = normal[1] * 0.001/normal.norm();
		//coefficients_->values[5] = normal[2] * 0.001/normal.norm();
	}

	void Primitive3Ellipse::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.axis_x = coefficients_->values[5];
		properties_.axis_y = coefficients_->values[6];
		properties_.axis_z = coefficients_->values[7];
		properties_.e0_x = coefficients_->values[8];
		properties_.e0_y = coefficients_->values[9];
		properties_.e0_z = coefficients_->values[10];

		Eigen::Vector3f e2(properties_.axis_x, properties_.axis_y, properties_.axis_z);
		Eigen::Vector3f e0(properties_.e0_x, properties_.e0_y, properties_.e0_z);
		Eigen::Vector3f e1 = e2.cross(e0).normalized();
		properties_.e1_x = e1(0);
		properties_.e1_y = e1(1);
		properties_.e1_z = e1(2);
	}
}