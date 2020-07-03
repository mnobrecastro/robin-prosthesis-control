#include "primitive3_cuboid.h"

namespace robin
{
	void Primitive3Cuboid::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		if (visualizeOnOff_)
			viewer->addCube(*coefficients_, "cube");
		for (auto plane : planes_) {
			plane->setVisualizeOnOff(false);
			plane->visualize(viewer);
		}
	}

	void Primitive3Cuboid::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		size_t n_planes(0), n_pts(cloud_->points.size());
		while (cloud_->points.size() > 0.3 * n_pts && n_planes < 3) {
			Primitive3Plane* plane(new Primitive3Plane());
			plane->fit(cloud, normals);
			*cloud_ += *(plane->getPointCloud());
			planes_.push_back(plane);
			++n_planes;
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Cuboid::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		if (!cloud_->points.empty()) {
			cloud_->clear();
		}
		if (!planes_.empty()) {
			for (auto plane : planes_) {
				delete plane;
			}
			planes_.clear();
		}

		size_t n_planes(0); int n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_planes < 3) {
			Primitive3Plane* plane(new Primitive3Plane());
			plane->fit(cloud, seg);
			*cloud_ += *(plane->getPointCloud());
			planes_.push_back(plane);
			++n_planes;
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
	bool Primitive3Cuboid::is_fit_valid()
	{
		//if (!cloud_->points.empty()) {
		//	/* x_min, x_max, y_min, y_max, z_min, z_max. */
		//	std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
		//	/* Checks the z-coordinate of the Primitive3Sphere center. */
		//	if (coefficients_->values[2] > ranges[4]) {
		//		return true;
		//	}
		//}
		//cloud_->points.clear();
		//coefficients_->values[0] = -0.001;
		//coefficients_->values[1] = -0.001;
		//coefficients_->values[2] = -0.001;
		//coefficients_->values[3] = 0.001;
		//return false;
		return true;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Cuboid::correct_coefficients()
	{
		/*//Plane coefficients [normal_x normal_y normal_z d]
		pcl::PointXYZ face_center(mean.x(), mean.y(), mean.z());
		pcl::PointXYZ cube_center(face_center.x + arr_coeffs_plane[0]->values[0] * 0.025, face_center.y + arr_coeffs_plane[0]->values[1] * 0.025, face_center.z + arr_coeffs_plane[0]->values[2] * 0.025);
		Eigen::Quaternionf quat;
		Eigen::Vector3f v1(0.0, 0.0, 1.0);
		Eigen::Vector3f v2(arr_coeffs_plane[0]->values[0], arr_coeffs_plane[0]->values[1], arr_coeffs_plane[0]->values[2]);
		quat.setFromTwoVectors(v1, v2);

		//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
		coefficients_primitive->values.push_back(cube_center.x); //Tx
		coefficients_primitive->values.push_back(cube_center.y); //Ty
		coefficients_primitive->values.push_back(cube_center.z); //Tz
		coefficients_primitive->values.push_back(quat.x()); //Qx
		coefficients_primitive->values.push_back(quat.y()); //Qy
		coefficients_primitive->values.push_back(quat.z()); //Qz
		coefficients_primitive->values.push_back(quat.w()); //Qw
		coefficients_primitive->values.push_back(0.050); //width
		coefficients_primitive->values.push_back(0.050); //height
		coefficients_primitive->values.push_back(0.050); //depth */
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Cuboid::update_properties()
	{
		/*properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.radius = coefficients_->values[3]; */
	}


}