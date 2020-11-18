#include "primitive3_line.h"

namespace robin
{
	Primitive3Line::Primitive3Line()
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
	}
	Primitive3Line::~Primitive3Line() {}
	
	void Primitive3Line::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addLine(*coefficients_, "line");
	}

	void Primitive3Line::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = 0.000;
		coefficients_->values[1] = 0.000;
		coefficients_->values[2] = 0.000;
		coefficients_->values[3] = 0.000;
		coefficients_->values[4] = 0.000;
		coefficients_->values[5] = 0.000;
	}


	void Primitive3Line::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_LINE);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_LINE);
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Line::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_LINE);
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
	bool Primitive3Line::is_fit_valid()
	{
		/* Not implemented yet. */
		return true;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Line::correct_coefficients()
	{
		pcl::PointXYZ point_on_axis(coefficients_->values[0], coefficients_->values[1], coefficients_->values[2]);
		pcl::PointXYZ axis_direction(coefficients_->values[3], coefficients_->values[4], coefficients_->values[5]);
		std::array<float, 2> arr(getPointCloudExtremes(*cloud_, point_on_axis, axis_direction));
		
		pcl::PointXYZ point_bottom;
		point_bottom.x = point_on_axis.x + arr[0] * axis_direction.x / normPointXYZ(axis_direction);
		point_bottom.y = point_on_axis.y + arr[0] * axis_direction.y / normPointXYZ(axis_direction);
		point_bottom.z = point_on_axis.z + arr[0] * axis_direction.z / normPointXYZ(axis_direction);
		pcl::PointXYZ bottom_top_direction;
		bottom_top_direction.x = (-arr[0] + arr[1]) * axis_direction.x / normPointXYZ(axis_direction);
		bottom_top_direction.y = (-arr[0] + arr[1]) * axis_direction.y / normPointXYZ(axis_direction);
		bottom_top_direction.z = (-arr[0] + arr[1]) * axis_direction.z / normPointXYZ(axis_direction);
		
		coefficients_->values[0] = point_bottom.x;
		coefficients_->values[1] = point_bottom.y;
		coefficients_->values[2] = point_bottom.z;
		coefficients_->values[3] = bottom_top_direction.x;
		coefficients_->values[4] = bottom_top_direction.y;
		coefficients_->values[5] = bottom_top_direction.z;
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Line::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.radius = coefficients_->values[3];
	}
}