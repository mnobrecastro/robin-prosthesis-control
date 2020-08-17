#include "primitive3_sphere.h"

namespace robin
{
	Primitive3Sphere::Primitive3Sphere()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 4-element SPHERE coefficients
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
	}
	Primitive3Sphere::~Primitive3Sphere() {}

	void Primitive3Sphere::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addSphere(*coefficients_, "sphere");
	}

	void Primitive3Sphere::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.001;
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
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Sphere::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_SPHERE);
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

		/* Checks if the cut sub-primitives are valid. */
		bool valid_subprims(true);
		if (!subprims_.empty()) {
			int i(0);
			while(valid_subprims) {
				if (subprims_[i]->getPointCloud()->empty()) {
					valid_subprims = false;
					continue;
				}
				++i;
			}
			if (valid_subprims) { return true; }
		}

		this->reset();
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


	///

	/* Receives a PointCloud cut by reference and fits a sub-primitive to it. */
	void Primitive3Sphere::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		Primitive3Circle* cut_prim(new Primitive3Circle);
		cut_prim->fit(cloud, false);
		*cloud_ += *cut_prim->getPointCloud();
		subprims_.push_back(cut_prim);
		std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud->size() << std::endl;
	}

	/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
	void Primitive3Sphere::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		Primitive3Circle* cut_prim(new Primitive3Circle);
		cut_prim->fit(cloud, seg);
		*cloud_ += *cut_prim->getPointCloud();
		subprims_.push_back(cut_prim);
		std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud->size() << std::endl;
	}

	void Primitive3Sphere::heuristic_laser_array_cross()
	{
		float sphere_radius(0.001);
		std::list<float> save_sphere_radius;

		size_t MOVING_AVG_SIZE(50);

		// Computing the boundaries of the line primitives
		//   [-]
		//   [-]
		// [-----]
		//   [-]
		//   [-]
		//   [-]

		std::array<float, 6> bounds_temp_h(getPointCloudRanges(*subprims_[0]->getPointCloud())); // 0 deg
		std::array<float, 2> bounds_horizontal({ bounds_temp_h[0] ,bounds_temp_h[1] });

		std::array<float, 6> bounds_temp_v(getPointCloudRanges(*subprims_[1]->getPointCloud())); // 90 deg
		std::array<float, 2> bounds_vertical({ bounds_temp_v[2] ,bounds_temp_v[3] });		


		// Finding the intersection '+' on the sphere
		int vertical_idx(-1), horizontal_idx(-1);
		if (bounds_vertical[0] <= 0.0 && 0.0 < bounds_vertical[1] && bounds_horizontal[0] <= 0.0 && 0.0 < bounds_horizontal[1]) {
			vertical_idx = 0;
			horizontal_idx = 0;
		}

		if (vertical_idx != -1 && horizontal_idx != -1) {
			std::cout << "vert_idx: " << vertical_idx << " hori_idx: " << horizontal_idx << std::endl;

			Eigen::Vector3f h_center(
				subprims_[0]->getCoefficients()->values[0],
				subprims_[0]->getCoefficients()->values[1], //0.0, //arr_coeffs_subprim_horizontal[horizontal_idx]->values[1],
				subprims_[0]->getCoefficients()->values[2]
			);
			float h_radius(subprims_[0]->getCoefficients()->values[6]);

			// Find each circle (cyl coef) center among the points in the subprimitive
			Eigen::Vector3f v_center(
				subprims_[1]->getCoefficients()->values[0], //0.0, //arr_coeffs_subprim_vertical[vertical_idx]->values[0],
				subprims_[1]->getCoefficients()->values[1],
				subprims_[1]->getCoefficients()->values[2]
			);
			float v_radius(subprims_[1]->getCoefficients()->values[6]);


			// Find the sphere center
			Eigen::Vector3f sphere_center(h_center.x(), v_center.y(), (h_center.z() + v_center.z()) / 2);

			// Sphere primitive parameters
			sphere_radius = std::sqrt(std::pow(h_radius, 2) + std::pow(sphere_center.y(), 2));

			// Sphere coefficients(center_x, center_y, center_z, r)
			coefficients_->values[0] = sphere_center.x(); //x
			coefficients_->values[1] = sphere_center.y(); //y
			coefficients_->values[2] = sphere_center.z(); //z
			coefficients_->values[3] = sphere_radius; //r
		}
		//else {
			//this->reset();
		//}
	}


}