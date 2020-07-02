#include "primitive3_plane.h"

namespace robin
{
	void Primitive3Plane::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		if(visualizeOnOff_)
			viewer->addPlane(*coefficients_, "plane");

		pcl::PointXYZ center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ center_e0(properties_.center_x + properties_.width / 2.0 * properties_.e0_x,
								properties_.center_y + properties_.width / 2.0 * properties_.e0_y,
								properties_.center_z + properties_.width / 2.0 * properties_.e0_z);
		viewer->addLine(center, center_e0, "plane_e0");
		pcl::PointXYZ center_e1(properties_.center_x + properties_.height / 2.0 * properties_.e1_x,
								properties_.center_y + properties_.height / 2.0 * properties_.e1_y,
								properties_.center_z + properties_.height / 2.0 * properties_.e1_z);
		viewer->addLine(center, center_e1, "plane_e1");
	}

	void Primitive3Plane::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE); //pcl::SACMODEL_NORMAL_PLANE ?
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Plane::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>* from_normals = dynamic_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg);
		if (from_normals != nullptr) {
			seg->setModelType(pcl::SACMODEL_NORMAL_PLANE);
			fit_sample_consensus_with_normals(cloud, seg);
		}
		else {
			seg->setModelType(pcl::SACMODEL_PLANE);
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
	bool Primitive3Plane::is_fit_valid()
	{
		return true;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Plane::correct_coefficients()
	{
		/* Nothing to correct. */
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Plane::update_properties()
	{
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(cloud_);
		Eigen::Vector4f mean(pca.getMean());
		Eigen::Matrix3f eigenvecs(pca.getEigenVectors());
		Eigen::Vector3f eigenvals(pca.getEigenValues());
		//Eigen::MatrixXf coeffs(pca.getCoefficients());
			
		Eigen::Vector3f e0 = eigenvecs.col(0);
		std::array<float, 2> e0_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(e0.x(), e0.y(), e0.z()))
		);

		Eigen::Vector3f e1 = eigenvecs.col(1);
		std::array<float, 2> e1_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(e1.x(), e1.y(), e1.z()))
		);

		properties_.center_x = mean.x();
		properties_.center_y = mean.y();
		properties_.center_z = mean.z();
		properties_.normal_x = coefficients_->values[0];
		properties_.normal_y = coefficients_->values[1];
		properties_.normal_z = coefficients_->values[2];
		properties_.e0_x = e0.x();
		properties_.e0_y = e0.y();
		properties_.e0_z = e0.z();
		properties_.e1_x = e1.x();
		properties_.e1_y = e1.y();
		properties_.e1_z = e1.z();
		properties_.width = e0_min_max[1] - e0_min_max[0];
		properties_.height = e1_min_max[1] - e1_min_max[0];
	}
}