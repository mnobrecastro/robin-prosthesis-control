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

		isempty_ = true;
	}
	Primitive3Sphere::~Primitive3Sphere() {}

	void Primitive3Sphere::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addSphere(*coefficients_, "sphere");
		viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 76.0 / 255.0, 0 / 255.0, 102 / 255.0, "sphere"); //153,0,204
		viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, "sphere");
	}

	void Primitive3Sphere::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.001;

		isempty_ = true;
	}

	void Primitive3Sphere::setCoefficients(std::vector<float> v)
	{
		if (v.size() == 4) {
			coefficients_->values[0] = v[0];
			coefficients_->values[1] = v[1];
			coefficients_->values[2] = v[2];
			coefficients_->values[3] = v[3];

			isempty_ = false;
		}
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
			/* Checks the z-coordinate of the Primitive3Cylinder center. */
			if (!(coefficients_->values[2] > ranges[4])) {
				this->reset();
				return false;
			}
		}
		else {
			this->reset();
			return false;
		}


		/* Checks if the cut sub-primitives are valid. */
		bool valid_subprims(true);
		if (!subprims_.empty()) {
			for (auto arr : subprims_) {
				for (auto sp : arr) {
					if (sp->getPointCloud()->empty()) {
						this->reset();
						return false;
					}
				}
			}
		}

		isempty_ = false;
		return true;
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
		std::vector<Primitive3d1*> subprim_arr;
		size_t n_subprims(0), n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_subprims < MAX_SUBPRIMS) {
			size_t cur_size(cloud->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud, false);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr.push_back(cut_prim);
			++n_subprims;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud->points.size() << std::endl;
		}
		subprims_.push_back(subprim_arr);
	}

	/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
	void Primitive3Sphere::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		std::vector<Primitive3d1*> subprim_arr;
		size_t n_subprims(0), n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_subprims < MAX_SUBPRIMS) {
			size_t cur_size(cloud->points.size());
			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud, seg);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr.push_back(cut_prim);
			++n_subprims;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud->points.size() << std::endl;
		}
		subprims_.push_back(subprim_arr);
	}

	void Primitive3Sphere::heuristic_laser_array_single()
	{
		/* WARNING!
		* This implementation does not guarantee a full estimation of the geometric primitive.
		*/
		float sphere_radius(0.001);

		// Computing the boundaries of the cut primitives
		// [   ][-----][   ][   ]
		std::vector<std::array<float, 2>> bounds_horizontal;
		for (int i(0); i < subprims_[0].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[0][i]->getPointCloud()));
			bounds_horizontal.push_back({ bounds_temp[0] ,bounds_temp[1] });
		}

		// Finding the "front face" spanning the center '-'
		int horizontal_idx(-1);
		for (int k1(0); k1 < bounds_horizontal.size(); ++k1) {
			if (bounds_horizontal[k1][0] <= 0.0 && 0.0 < bounds_horizontal[k1][1]) {
				horizontal_idx = k1;
				break;
			}
		}

		if (horizontal_idx != -1) {
			std::cout << " hori_idx: " << horizontal_idx << std::endl;

			// Find each circle (cyl coef) center among the points in the horizontal subprimitive
			Eigen::Vector3f h_center(
				subprims_[0][0]->getCoefficients()->values[0],
				subprims_[0][0]->getCoefficients()->values[1], //0.0, //arr_coeffs_subprim_horizontal[horizontal_idx]->values[1],
				subprims_[0][0]->getCoefficients()->values[2]
			);
			float h_radius(subprims_[0][0]->getCoefficients()->values[6]);

			// Find the sphere center
			Eigen::Vector3f sphere_center(h_center.x(), h_center.y(), h_center.z());

			// Sphere primitive parameters
			sphere_radius = h_radius;

			// Sphere coefficients(center_x, center_y, center_z, r)
			coefficients_->values[0] = sphere_center.x(); //x
			coefficients_->values[1] = sphere_center.y(); //y
			coefficients_->values[2] = sphere_center.z(); //z
			coefficients_->values[3] = sphere_radius; //r
		}
	}

	void Primitive3Sphere::heuristic_laser_array_cross()
	{
		float sphere_radius(0.001);

		// Computing the boundaries of the cut primitives
		//        [-]
		//        [-]
		// [   ][-----][   ][   ]
		//        [-]
		//        [-]
		//        [-]
		std::vector<std::array<float, 2>> bounds_horizontal;
		for (int i(0); i < subprims_[0].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[0][i]->getPointCloud()));
			bounds_horizontal.push_back({ bounds_temp[0] ,bounds_temp[1] });
		}
		std::vector<std::array<float, 2>> bounds_vertical;
		for (int i(0); i < subprims_[1].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[1][i]->getPointCloud()));
			bounds_vertical.push_back({ bounds_temp[2] ,bounds_temp[3] });
		}

		// Finding the "front face" spaned by the intersection '+'
		int vertical_idx(-1), horizontal_idx(-1);
		for (int k1(0); k1 < bounds_vertical.size(); ++k1) {
			for (int k2(0); k2 < bounds_horizontal.size(); ++k2) {
				if (bounds_vertical[k1][0] <= 0.0 && 0.0 < bounds_vertical[k1][1] && bounds_horizontal[k2][0] <= 0.0 && 0.0 < bounds_horizontal[k2][1]) {
					vertical_idx = k1;
					horizontal_idx = k2;
					break;
				}
			}
		}

		if (vertical_idx != -1 && horizontal_idx != -1) {
			std::cout << "vert_idx: " << vertical_idx << " hori_idx: " << horizontal_idx << std::endl;
			// Find each circle (cyl coef) center among the points in the horizontal subprimitive
			Eigen::Vector3f h_center(
				subprims_[0][0]->getCoefficients()->values[0],
				subprims_[0][0]->getCoefficients()->values[1], // = 0.0
				subprims_[0][0]->getCoefficients()->values[2]
			);
			float h_radius(subprims_[0][0]->getCoefficients()->values[6]);
			// Find each circle (cyl coef) center among the points in the vectical subprimitive
			Eigen::Vector3f v_center(
				subprims_[1][0]->getCoefficients()->values[0], // = 0.0
				subprims_[1][0]->getCoefficients()->values[1],
				subprims_[1][0]->getCoefficients()->values[2]
			);
			float v_radius(subprims_[1][0]->getCoefficients()->values[6]);

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
	}

	void Primitive3Sphere::heuristic_laser_array_star()
	{
		/* Not implemented yet. */
	}
}