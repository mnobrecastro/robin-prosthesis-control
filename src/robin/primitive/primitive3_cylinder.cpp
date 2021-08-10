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
		viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 76.0 / 255.0, 0 / 255.0, 102 / 255.0, "cylinder"); //153,0,204
		viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, "cylinder");
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

	void Primitive3Cylinder::setCoefficients(std::vector<float> v)
	{
		if (v.size() == 7) {
			coefficients_->values[0] = v[0];
			coefficients_->values[1] = v[1];
			coefficients_->values[2] = v[2];
			coefficients_->values[3] = v[3];
			coefficients_->values[4] = v[4];
			coefficients_->values[5] = v[5];
			coefficients_->values[6] = v[6];
		}
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
			/* pcl::SACSegmentation<pcl::PointXYZ> for CYLINDER
			 * NOT IMPLEMENTED IN THE POINT CLOUD LIBRARY!
			 * pcl\segmentation\impl\sac_segmentation.hpp */
			
			// Create a new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>
			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>* segn(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);
			segn->setNormalDistanceWeight(0.001);
			segn->setModelType(seg->getModelType());
			segn->setOptimizeCoefficients(seg->getOptimizeCoefficients());
			segn->setMethodType(seg->getMethodType());			
			segn->setMaxIterations(seg->getMaxIterations());
			segn->setDistanceThreshold(seg->getDistanceThreshold());
			double min_lim(0.0), max_lim(0.0);
			seg->getRadiusLimits(min_lim, max_lim);
			segn->setRadiusLimits(min_lim, max_lim);

			fit_sample_consensus_with_normals(cloud, segn);
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
		properties_.center_x = coefficients_->values[0] + coefficients_->values[3] / 2;
		properties_.center_y = coefficients_->values[1] + coefficients_->values[4] / 2;
		properties_.center_z = coefficients_->values[2] + coefficients_->values[5] / 2;
		properties_.axis_x = coefficients_->values[3] / 2;
		properties_.axis_y = coefficients_->values[4] / 2;
		properties_.axis_z = coefficients_->values[5] / 2;
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

		//subprims_.push_back(subprim_arr_ellipse);
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
					subprims_[0][horizontal_idx]->getCoefficients()->values[1], //0.0,
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
					subprims_[1][vertical_idx]->getCoefficients()->values[0], //0.0,
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
		float cylinder_radius(0.001), cylinder_height(0.001);
		std::list<float> save_cylinder_radius, save_cylinder_height;

		size_t MOVING_AVG_SIZE(50);
		
		// Computing the boundaries of the circle primitives
		//   \ [-] /
		//    \[-]/
		//	 [-----][--------]
		//    /[-]\
		//   / [-] \
		//  /  [-]  \
		
		std::array<pcl::PointXYZ, 8> keypoints;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_face(new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<std::array<float, 2>> bounds_0; int idx_0(-1); // 0 deg
		for (int i(0); i < subprims_[0].size(); ++i) {
			pcl::PointXYZ min, max;
			std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[0][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 0, 0), min, max));
			bounds_0.push_back({ bounds_temp[0] ,bounds_temp[1] });
			*cloud_suprims += *subprims_[0][i]->getPointCloud();

			// Finding the front cube_face spaned by the intersection '+'
			if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
				idx_0 = i;
				keypoints[4] = min;
				keypoints[0] = max;
				if (subprims_[0][i]->getCoefficients()->values.size() == 7) {
					// Subprimitive is a Circle (cyl coef)
					cloud_face->push_back(min);
					cloud_face->push_back(max);
				}
			}
		}
		std::vector<std::array<float, 2>> bounds_1; int idx_1(-1); // 45 deg
		for (int i(0); i < subprims_[1].size(); ++i) {
			pcl::PointXYZ min, max;
			std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[1][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 1, 0), min, max));
			bounds_1.push_back({ bounds_temp[0] ,bounds_temp[1] });
			*cloud_suprims += *subprims_[1][i]->getPointCloud();

			// Finding the front cube_face spaned by the intersection '+'
			if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
				idx_1 = i;
				keypoints[5] = min;
				keypoints[1] = max;
				if (subprims_[1][i]->getCoefficients()->values.size() == 7) {
					// Subprimitive is a Circle (cyl coef)
					cloud_face->push_back(min);
					cloud_face->push_back(max);
				}
			}
		}
		std::vector<std::array<float, 2>> bounds_2; int idx_2(-1); // 90 deg
		for (int i(0); i < subprims_[2].size(); ++i) {
			pcl::PointXYZ min, max;
			std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[2][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 1, 0), min, max));
			bounds_2.push_back({ bounds_temp[0] ,bounds_temp[1] });
			*cloud_suprims += *subprims_[2][i]->getPointCloud();

			// Finding the front cube_face spaned by the intersection '+'
			if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
				idx_2 = i;
				keypoints[6] = min;
				keypoints[2] = max;
				if (subprims_[2][i]->getCoefficients()->values.size() == 7) {
					// Subprimitive is a Circle (cyl coef)
					cloud_face->push_back(min);
					cloud_face->push_back(max);
				}
			}
		}
		std::vector<std::array<float, 2>> bounds_3; int idx_3(-1); // 135 deg
		for (int i(0); i < subprims_[3].size(); ++i) {
			pcl::PointXYZ min, max;
			std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[3][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(-1, 1, 0), min, max));
			bounds_3.push_back({ bounds_temp[0] ,bounds_temp[1] });
			*cloud_suprims += *subprims_[3][i]->getPointCloud();

			// Finding the front cube_face spaned by the intersection '+'
			if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
				idx_3 = i;
				keypoints[7] = min;
				keypoints[3] = max;
				if (subprims_[3][i]->getCoefficients()->values.size() == 7) {
					// Subprimitive is a Circle (cyl coef)
					cloud_face->push_back(min);
					cloud_face->push_back(max);
				}
			}
		}
		/*
		if (idx_0 != -1 && idx_1 != -1 && idx_2 != -1 && idx_3 != -1) {
			std::cout << "idx_0: " << idx_0 << " idx_1: " << idx_1 << " idx_2: " << idx_2 << " idx_3: " << idx_3 << std::endl;

			// Find the centroid of the points in the line primitive
			bool found(false);
			Eigen::Vector3f va_dir; // first vector to be found by the heuristic
			for (int i(0); i < keypoints.size(); ++i) {
				Eigen::Vector3f kp, kp_prev, kp_next;
				kp(0) = keypoints[i].x;
				kp(1) = keypoints[i].y;
				kp(2) = keypoints[i].z;
				if (i == 0) {
					kp_prev(0) = keypoints[keypoints.size() - 1].x;
					kp_prev(1) = keypoints[keypoints.size() - 1].y;
					kp_prev(2) = keypoints[keypoints.size() - 1].z;
				}
				else {
					kp_prev(0) = keypoints[i - 1].x;
					kp_prev(1) = keypoints[i - 1].y;
					kp_prev(2) = keypoints[i - 1].z;
				}
				if (i == keypoints.size() - 1) {
					kp_next(0) = keypoints[0].x;
					kp_next(1) = keypoints[0].y;
					kp_next(2) = keypoints[0].z;
				}
				else {
					kp_next(0) = keypoints[i + 1].x;
					kp_next(1) = keypoints[i + 1].y;
					kp_next(2) = keypoints[i + 1].z;
				}

				// Check for parallel vectors
				Eigen::Vector3f v_prev = kp - kp_prev;
				Eigen::Vector3f v_next = kp_next - kp;
				float angle = std::acos(v_prev.dot(v_next) / (v_prev.norm() * v_next.norm()));

				// Estimate the direction.
				if (angle < 5.0 * M_PI / 180.0) {
					va_dir = kp_next - kp_prev;
					va_dir.normalize();
					std::cout << "Angle between vectors: " << angle * 180.0 / M_PI << std::endl;
					break;
				}
			}

			Eigen::Vector3f face_normal, v0, v1;
			v0 = Eigen::Vector3f(keypoints[0].x, keypoints[0].y, keypoints[0].z) - Eigen::Vector3f(keypoints[4].x, keypoints[4].y, keypoints[4].z);
			v1 = Eigen::Vector3f(keypoints[2].x, keypoints[2].y, keypoints[2].z) - Eigen::Vector3f(keypoints[6].x, keypoints[6].y, keypoints[6].z);
			face_normal = v0.cross(v1);
			face_normal.normalize();

			// Find the cube_face center
			Eigen::Vector3f face_center(0.0, 0.0, 0.0);
			for (auto p : cloud_face->points) {
				face_center(0) += p.x;
				face_center(1) += p.y;
				face_center(2) += p.z;
			}
			face_center /= cloud_face->size();

			// Remaining direction vector
			Eigen::Vector3f vb_dir = face_normal.cross(va_dir);
			vb_dir.normalize();

			// Cube primitive parameters

			std::array<float, 2> bounds_va(getPointCloudExtremes(*cloud_face, pcl::PointXYZ(face_center.x(), face_center.y(), face_center.z()), pcl::PointXYZ(va_dir.x(), va_dir.y(), va_dir.z())));
			cube_width = bounds_va[1] - bounds_va[0];
			//cube_width = moving_average(cube_width, save_cube_width, MOVING_AVG_SIZE, EXPONENTIAL);
			std::array<float, 2> bounds_vb(getPointCloudExtremes(*cloud_face, pcl::PointXYZ(face_center.x(), face_center.y(), face_center.z()), pcl::PointXYZ(vb_dir.x(), vb_dir.y(), vb_dir.z())));
			cube_height = bounds_vb[1] - bounds_vb[0];
			//cube_height = moving_average(cube_height, save_cube_height, MOVING_AVG_SIZE, EXPONENTIAL);
			std::array<float, 2> bounds_normal(getPointCloudExtremes(*cloud_suprims, pcl::PointXYZ(face_center.x(), face_center.y(), face_center.z()), pcl::PointXYZ(face_normal.x(), face_normal.y(), face_normal.z())));
			cube_depth = bounds_normal[1] - bounds_normal[0];
			//cube_depth = moving_average(cube_depth, save_cube_depth, MOVING_AVG_SIZE, EXPONENTIAL);
			Eigen::Vector3f cube_center = face_center - face_normal * cube_depth / 2;

			Eigen::Matrix3f mori;
			mori(0, 0) = face_normal.x();
			mori(1, 0) = face_normal.y();
			mori(2, 0) = face_normal.z();

			mori(0, 1) = va_dir.x();
			mori(1, 1) = va_dir.y();
			mori(2, 1) = va_dir.z();

			mori(0, 2) = vb_dir.x();
			mori(1, 2) = vb_dir.x();
			mori(2, 2) = vb_dir.x();

			Eigen::Quaternionf quat(mori);

			// Cylinder coefficients(point_x, point_y, point_z, axis_x, axis_y, axis_z, radius)
			coefficients_->values[0] = cyl_bottom_center.x(); //point_x
			coefficients_->values[1] = cyl_bottom_center.y(); //point_y
			coefficients_->values[2] = cyl_bottom_center.z(); //point_z
			coefficients_->values[3] = dir.x(); //axis_x
			coefficients_->values[4] = dir.y(); //axis_y
			coefficients_->values[5] = dir.z(); //axis_z
			coefficients_->values[6] = cylinder_radius; //radius

			//
			plot_[0] = cube_center.x();
			plot_[1] = cube_center.y();
			plot_[2] = cube_center.z();
			plot_[3] = face_normal.x();
			plot_[4] = face_normal.y();
			plot_[5] = face_normal.z();
			plot_[6] = va_dir.x();
			plot_[7] = va_dir.y();
			plot_[8] = va_dir.z();
			plot_[9] = vb_dir.x();
			plot_[10] = vb_dir.y();
			plot_[11] = vb_dir.z();

			if (cube_width >= cube_height && cube_width >= cube_depth) {
				properties_.axis_x = va_dir.x();
				properties_.axis_y = va_dir.y();
				properties_.axis_z = va_dir.z();
			}
			else if (cube_height >= cube_width && cube_height >= cube_depth) {
				properties_.axis_x = vb_dir.x();
				properties_.axis_y = vb_dir.y();
				properties_.axis_z = vb_dir.z();
			}
			else if (cube_depth > 0.005 && cube_depth >= cube_width && cube_depth >= cube_height) {
				properties_.axis_x = face_normal.x();
				properties_.axis_y = face_normal.y();
				properties_.axis_z = face_normal.z();
			}
			//

			std::cout << "\t" << cube_width << " " << cube_height << " " << cube_depth << std::endl;
		}*/
	}
}