#include "primitive3_cylinder.h"

namespace robin
{
	Primitive3Cylinder::Primitive3Cylinder()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 7-element cylinder coefficients
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
		coefficients_->values.push_back(0.0); //4
		coefficients_->values.push_back(0.0); //5
		coefficients_->values.push_back(0.0); //6

	}
	Primitive3Cylinder::~Primitive3Cylinder() {}
	
	void Primitive3Cylinder::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addCylinder(*coefficients_, "cylinder");
	}

	void Primitive3Cylinder::reset()
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

	void Primitive3Cylinder::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CYLINDER);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CYLINDER);
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();			
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Cylinder::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_CYLINDER);
		if (typeid(*seg) == typeid(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>)) {
			fit_sample_consensus_with_normals(cloud, seg);
		}
		else {
			/* NOT IMPLEMENTED IN THE POINT CLOUD LIBRARY!
			 * pcl\segmentation\impl\sac_segmentation.hpp */
			std::cout << "Primitive3Cylinder only working with 'pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>'." << std::endl; exit(-1);
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
	bool Primitive3Cylinder::is_fit_valid()
	{
		if (!cloud_->points.empty()) {
			/* x_min, x_max, y_min, y_max, z_min, z_max. */
			std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
			/* Checks the z-coordinate of the Primitive3Cylinder center. */
			if (coefficients_->values[2] > ranges[4]) {
				return true;
			}
		}

		/* Checks if the cut sub-primitives are valid. */
		bool valid_subprims(true);
		if (!subprims_.empty()) {
			for (auto arr : subprims_) {
				for (auto sp : arr) {
					if (sp->getPointCloud()->empty()) {
						valid_subprims = false;
						break;
					}
				}
				if (!valid_subprims) { break; }
			}
			if (valid_subprims) { return true; }
		}

		this->reset();
		return false;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Cylinder::correct_coefficients()
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
		//coefficients_->values[6] = coefficients_->values[6];
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Cylinder::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.axis_x = coefficients_->values[3];
		properties_.axis_y = coefficients_->values[4];
		properties_.axis_z = coefficients_->values[5];
		properties_.radius = coefficients_->values[6];
	}

	///
	
	/* Receives a PointCloud cut by reference and fits a sub-primitive to it. */
	void Primitive3Cylinder::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_circle;
		size_t n_subprims_circle(0), n_pts_circle(cloud_circle->points.size());
		while (cloud_circle->points.size() > 0.3 * n_pts_circle && n_subprims_circle < MAX_SUBPRIMS) {
			size_t cur_size(cloud_circle->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud_circle, false);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_circle.push_back(cut_prim);
			++n_subprims_circle;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud_circle->size() << std::endl;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_line;
		size_t n_subprims_line(0), n_pts_line(cloud_line->points.size());
		while (cloud_line->points.size() > 0.3 * n_pts_line && n_subprims_line < MAX_SUBPRIMS) {
			size_t cur_size(cloud_line->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud_line, false);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_line.push_back(cut_prim);
			++n_subprims_line;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud_line->size() << std::endl;
		}

		if (cloud_circle->points.size() < cloud_line->points.size()) {
			subprims_.push_back(subprim_arr_circle);
		} else {
			subprims_.push_back(subprim_arr_line);
		}		
	}

	/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
	void Primitive3Cylinder::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_circle;
		size_t n_subprims_circle(0), n_pts_circle(cloud_circle->points.size());
		while (cloud_circle->points.size() > 0.3 * n_pts_circle && n_subprims_circle < MAX_SUBPRIMS) {
			size_t cur_size(cloud_circle->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud_circle, seg);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_circle.push_back(cut_prim);
			++n_subprims_circle;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud_circle->size() << std::endl;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_line;
		size_t n_subprims_line(0), n_pts_line(cloud_line->points.size());
		while (cloud_line->points.size() > 0.3 * n_pts_line && n_subprims_line < MAX_SUBPRIMS-1) {
			size_t cur_size(cloud_line->points.size());

			Primitive3Line* cut_prim(new Primitive3Line);
			cut_prim->fit(cloud_line, seg);
			*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_line.push_back(cut_prim);
			++n_subprims_line;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cloud_line->size() << std::endl;
		}

		if (cloud_circle->points.size() < cloud_line->points.size()) {
			subprims_.push_back(subprim_arr_circle);
		}
		else {
			subprims_.push_back(subprim_arr_line);
		}
	}

	void Primitive3Cylinder::heuristic_laser_array_single()
	{
		/* Not implemented yet. */
	}

	void Primitive3Cylinder::heuristic_laser_array_cross()
		//void michelangelo::heuristicCylinder(pcl::ModelCoefficients::Ptr& coefficients_primitive, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& arr_cloud_subprim_vertical, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& arr_cloud_subprim_horizontal, std::vector<pcl::ModelCoefficients::Ptr>& arr_coeffs_subprim_vertical, std::vector<pcl::ModelCoefficients::Ptr>& arr_coeffs_subprim_horizontal)
	{
		float cylinder_radius(0.001), cylinder_height(0.001);
		std::list<float> save_cylinder_radius, save_cylinder_height;

		size_t MOVING_AVG_SIZE(50);

		// Computing the boundaries of the line primitives
		//   [-]
		//   [-]
		// [-----]
		//   [-]
		//   [-]
		//   [-]
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

		// Finding the front cube_face spaned by the intersection '+'
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

			// Find the centroid/center among the points of each subprimitives
			Eigen::Vector3f false_bottom, dir, v_center(0.0, 0.0, 0.0), h_center(0.0, 0.0, 0.0);
			float v_radius, h_radius;
			bool has_line(false);

			if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 6 && !has_line) {
				// Subprimitive is a Line
				false_bottom = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					subprims_[0][horizontal_idx]->getCoefficients()->values[1],
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);
				dir = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[3],
					subprims_[0][horizontal_idx]->getCoefficients()->values[4],
					subprims_[0][horizontal_idx]->getCoefficients()->values[5]
				);
				has_line = true;
			}
			else if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 7) {
				// Subprimitive is a Circle (cyl coef)
				h_center = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					0.0, //arr_coeffs_subprim_horizontal[horizontal_idx]->values[1],
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);
				h_radius = subprims_[0][horizontal_idx]->getCoefficients()->values[6];
			}

			if (subprims_[1][vertical_idx]->getCoefficients()->values.size() == 6 && !has_line) {
				// Subprimitive is a Line
				false_bottom = Eigen::Vector3f(
					subprims_[1][vertical_idx]->getCoefficients()->values[0],
					subprims_[1][vertical_idx]->getCoefficients()->values[1],
					subprims_[1][vertical_idx]->getCoefficients()->values[2]
				);
				dir = Eigen::Vector3f(
					subprims_[1][vertical_idx]->getCoefficients()->values[3],
					subprims_[1][vertical_idx]->getCoefficients()->values[4],
					subprims_[1][vertical_idx]->getCoefficients()->values[5]
				);
				has_line = true;
			}
			else if (subprims_[1][vertical_idx]->getCoefficients()->values.size() == 7) {
				// Subprimitive is a Circle (cyl coef)
				v_center = Eigen::Vector3f(
					0.0, //arr_coeffs_subprim_vertical[vertical_idx]->values[0],
					subprims_[1][vertical_idx]->getCoefficients()->values[1],
					subprims_[1][vertical_idx]->getCoefficients()->values[2]
				);
				v_radius = subprims_[1][vertical_idx]->getCoefficients()->values[6];
			}

			// Find the cylinder bottom center
			Eigen::Vector3f center, cyl_bottom_center;
			if (v_center.isZero() ^ h_center.isZero()) { //XOR
				if (v_center.isZero()) {
					center = h_center;
					cylinder_radius = h_radius;
				}
				else if (h_center.isZero()) {
					center = v_center;
					cylinder_radius = v_radius;
				}
				//cylinder_radius = moving_average(cylinder_radius, save_cylinder_radius, MOVING_AVG_SIZE, EXPONENTIAL);

				Eigen::Vector3f vec(false_bottom.x() - center.x(), false_bottom.y() - center.y(), false_bottom.z() - center.z());
				cyl_bottom_center = Eigen::Vector3f(
					center.x() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.x(),
					center.y() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.y(),
					center.z() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.z()
				);

				// Cylinder primitive parameters
				cylinder_height = dir.norm();
				//cylinder_height = moving_average(cylinder_height, save_cylinder_height, MOVING_AVG_SIZE, EXPONENTIAL);
			}

			// Cylinder coefficients(point_x, point_y, point_z, axis_x, axis_y, axis_z, radius)
			coefficients_->values[0] = cyl_bottom_center.x(); //point_x
			coefficients_->values[1] = cyl_bottom_center.y(); //point_y
			coefficients_->values[2] = cyl_bottom_center.z(); //point_z
			coefficients_->values[3] = dir.x(); //axis_x
			coefficients_->values[4] = dir.y(); //axis_y
			coefficients_->values[5] = dir.z(); //axis_z
			coefficients_->values[6] = cylinder_radius; //radius
		}
	}


	void Primitive3Cylinder::heuristic_laser_array_star()
	{
		/* Not implemented yet. */
	}
}