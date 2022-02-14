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

		isempty_ = true;
	}

	void Primitive3Cylinder::setCoefficients(std::vector<float> v)
	{
		if (v.size() == 7) {
			if (std::sqrt(
				coefficients_->values[3] * coefficients_->values[3] +
				coefficients_->values[4] * coefficients_->values[4] +
				coefficients_->values[5] * coefficients_->values[5]) > 0 &&
				coefficients_->values[6] > 0) {
				coefficients_->values[0] = v[0];
				coefficients_->values[1] = v[1];
				coefficients_->values[2] = v[2];
				coefficients_->values[3] = v[3];
				coefficients_->values[4] = v[4];
				coefficients_->values[5] = v[5];
				coefficients_->values[6] = v[6];

				isempty_ = false;
			}
			else
				std::cerr << "ERROR: The set of coefficients is not valid." << std::endl;
		}
		else
			std::cerr << "ERROR: The set of coefficients is not valid." << std::endl;
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
			if (!(coefficients_->values[2] > ranges[4])) {
				this->reset();
				return false;
			}
		} else {
			this->reset();
			return false;
		}

		/* Checks if the components of the direction vector and the radius are valid. */
		if (!(std::sqrt(
			coefficients_->values[3] * coefficients_->values[3] +
			coefficients_->values[4] * coefficients_->values[4] +
			coefficients_->values[5] * coefficients_->values[5]) > 0 &&
			coefficients_->values[6] > 0))
		{
			this->reset();
			return false;
		}

		isempty_ = false;
		return true;
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
#ifndef USE_ELLIPSE
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_circle;
		size_t n_subprims_circle(0), n_pts_circle(cloud_circle->points.size());
		while (cloud_circle->points.size() > 0.3 * n_pts_circle && n_subprims_circle < MAX_SUBPRIMS) {
			size_t cur_size(cloud_circle->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud_circle, false);
			//*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_circle.push_back(cut_prim);
			++n_subprims_circle;
			std::cout << "\t" << "CIRCLE: " << cut_prim->getPointCloud()->size() << "/" << cur_size << std::endl;
		}
#else
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ellipse(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_ellipse;
		size_t n_subprims_ellipse(0), n_pts_ellipse(cloud_ellipse->points.size());
		while (cloud_ellipse->points.size() > 0.3 * n_pts_ellipse && n_subprims_ellipse < MAX_SUBPRIMS) {
			size_t cur_size(cloud_ellipse->points.size());

			Primitive3Ellipse* cut_prim(new Primitive3Ellipse);
			cut_prim->fit(cloud_ellipse, false);
			//*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_ellipse.push_back(cut_prim);
			++n_subprims_ellipse;
			std::cout << "\t" << "ELLIPSE: " << cut_prim->getPointCloud()->size() << "/" << cur_size << std::endl;
		}
#endif
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_line;
		size_t n_subprims_line(0), n_pts_line(cloud_line->points.size());
		while (cloud_line->points.size() > 0.3 * n_pts_line && n_subprims_line < MAX_SUBPRIMS) {
			size_t cur_size(cloud_line->points.size());

			Primitive3Circle* cut_prim(new Primitive3Circle);
			cut_prim->fit(cloud_line, false);
			//*cloud_ += *cut_prim->getPointCloud();
			subprim_arr_line.push_back(cut_prim);
			++n_subprims_line;
			std::cout << "\t" << "LINE: " << cut_prim->getPointCloud()->size() << "/" << cur_size << std::endl;
		}


#ifndef USE_ELLIPSE
		if (cloud_circle->points.size() < cloud_line->points.size()) {
			subprims_.push_back(subprim_arr_circle);
		} else {
			subprims_.push_back(subprim_arr_line);
		}
#else
		// A successful fitting presents a very reduced amount of outliers:
		// i.e. lower the number of outlier points, higher the fitting rate.
		if (cloud_ellipse->points.size() == n_pts_ellipse) {
			// (when the ellipse fitting fails, take the line model)
			subprims_.push_back(subprim_arr_line);
		}
		//else if (cloud_ellipse->points.size() < n_pts_ellipse) {
		else {
			// (else pick the ellipse model)
			subprims_.push_back(subprim_arr_ellipse);
		}
#endif
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
			subprim_arr_circle.push_back(cut_prim);
			++n_subprims_circle;
			std::cout << "\t" << "CIRCLE: " << cut_prim->getPointCloud()->points.size() << "/" << cur_size << std::endl;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_line;
		size_t n_subprims_line(0), n_pts_line(cloud_line->points.size());
		while (cloud_line->points.size() > 0.3 * n_pts_line && n_subprims_line < MAX_SUBPRIMS) {
			size_t cur_size(cloud_line->points.size());

			Primitive3Line* cut_prim(new Primitive3Line);
			cut_prim->fit(cloud_line, seg);
			subprim_arr_line.push_back(cut_prim);
			++n_subprims_line;
			std::cout << "\t" << "LINE: " << cut_prim->getPointCloud()->points.size() << "/" << cur_size << std::endl;
		}

#ifdef USE_ELLIPSE
		seg->setDistanceThreshold(0.010); // ellipse only := 0.005
		seg->setRadiusLimits(0.025, 0.25); // ellipse only := [0.025, 0.5]

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ellipse(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		std::vector<Primitive3d1*> subprim_arr_ellipse;
		size_t n_subprims_ellipse(0), n_pts_ellipse(cloud_ellipse->points.size());
		while (cloud_ellipse->points.size() > 0.3 * n_pts_ellipse && n_subprims_ellipse < MAX_SUBPRIMS) {
			size_t cur_size(cloud_ellipse->points.size());

			Primitive3Ellipse* cut_prim(new Primitive3Ellipse);
			cut_prim->fit(cloud_ellipse, seg);
			subprim_arr_ellipse.push_back(cut_prim);
			++n_subprims_ellipse;
			std::cout << "\t" << "ELLIPSE: " << cut_prim->getPointCloud()->points.size() << "/" << cur_size << std::endl;
		}
#endif

#ifndef USE_ELLIPSE
		if (cloud_circle->points.size() < cloud_line->points.size()) {
			subprims_.push_back(subprim_arr_circle);
		}
		else {
			subprims_.push_back(subprim_arr_line);
		}
#else
		// A successful fitting presents a very reduced amount of outliers:
		// i.e. lower the number of outlier points, higher the fitting rate.
		if (cloud_ellipse->points.size() == n_pts_ellipse) {
			// (when the ellipse fitting fails, take the line model)
			subprims_.push_back(subprim_arr_line);
		}
		//else if (cloud_ellipse->points.size() < n_pts_ellipse) {
		else {
			// (else pick the ellipse model)
			subprims_.push_back(subprim_arr_ellipse);
		}
#endif
		return;
	}

	void Primitive3Cylinder::heuristic_laser_array_single()
	{
		/* WARNING!
		 * This implementation does not guarantee a full estimation of the geometric primitive.
		 */
		float radius(0.001);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

		// Computing the boundaries of the cut primitives
		//
		// [   ][-----][   ][   ]
		//

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

			// Find the centroid/center among the points of each subprimitives
			Eigen::Vector3f center(0.0, 0.0, 0.0), dir(0.0, 1.0, 0.0);

			// Horizontal primitive
#ifndef USE_ELLIPSE
			if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 7) {
				// Subprimitive is a Circle (cyl coef)
				center = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					subprims_[0][horizontal_idx]->getCoefficients()->values[1], //0.0
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);
				dir = dir; // Not using the subprim's normal coefs
				radius = subprims_[0][horizontal_idx]->getCoefficients()->values[6];

				*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
				std::cout << "CIRCLE\n" << std::endl;
			}
#else
			if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 11) {
				// Subprimitive is an Ellipse
				center = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					subprims_[0][horizontal_idx]->getCoefficients()->values[1], //0.0
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);

				// Pick the largest semi-minor axes (sma) lenght
				float sma_lenght(0.0);
				if (subprims_[0][horizontal_idx]->getCoefficients()->values[3] >= subprims_[0][horizontal_idx]->getCoefficients()->values[4]) {
					sma_lenght = subprims_[0][horizontal_idx]->getCoefficients()->values[3];
					radius = subprims_[0][horizontal_idx]->getCoefficients()->values[4];
				} else {
					sma_lenght = subprims_[0][horizontal_idx]->getCoefficients()->values[4];
					radius = subprims_[0][horizontal_idx]->getCoefficients()->values[3];
				}
				dir = dir; // Not using the subprim's normal coefs

				if (false) {
					/* DEPRECATED */

					// Calculate the Ellipse tilt angle 'th', where 
					float th = std::acos(1 / (sma_lenght / radius));

					// Normal axis of the Ellipse
					Eigen::Vector3f n_axis;
					n_axis = Eigen::Vector3f(
						subprims_[0][horizontal_idx]->getCoefficients()->values[5],
						subprims_[0][horizontal_idx]->getCoefficients()->values[6],
						subprims_[0][horizontal_idx]->getCoefficients()->values[7]
					);
					dir = n_axis;
					//n_axis = dir;
					// X-axis of the Ellipse
					Eigen::Vector3f x_axis(
						subprims_[0][horizontal_idx]->getCoefficients()->values[8],
						subprims_[0][horizontal_idx]->getCoefficients()->values[9],
						subprims_[0][horizontal_idx]->getCoefficients()->values[10]
					);
					// Y-axis of the Ellipse
					Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();

					// Create the rotation matrix for the Ellipse's local reference frame
					Eigen::Matrix3f Frame;
					Frame << x_axis(0), y_axis(0), n_axis(0),
						x_axis(1), y_axis(1), n_axis(1),
						x_axis(2), y_axis(2), n_axis(2);

					// Calcular the Euler parameters to rotate the local frame about it's X-axis by the amount of tilt angle 'th' 
					float e0(std::cos(th / 2.0));
					Eigen::Vector3f e = x_axis * std::sin(th / 2.0);
					Eigen::Matrix3f e_skewsim;
					e_skewsim << 0.0, -e(2), e(1),
						e(2), 0.0, -e(0),
						-e(1), e(0), 0.0;
					Eigen::Matrix3f A = (2 * e0 * e0 - 1) * Eigen::Matrix3f::Identity() + 2 * (e * e.transpose() + e0 * e_skewsim);

					// Rotate the local frame
					Eigen::Matrix3f Rot = A * Frame;
					/*// Select the updated Z-axis
					dir = Eigen::Vector3f(Rot(0, 2), Rot(1, 2), Rot(2, 2));*/
				}				

				*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
				std::cout << "ELLIPSE\n" << std::endl;
			}
#endif
			*cloud_ = *cloud_suprims;

			// Cylinder coefficients(point_x, point_y, point_z, axis_x, axis_y, axis_z, radius)
			coefficients_->values[0] = center.x(); //point_x
			coefficients_->values[1] = center.y(); //point_y
			coefficients_->values[2] = center.z(); //point_z
			coefficients_->values[3] = dir.x(); //axis_x
			coefficients_->values[4] = dir.y(); //axis_y
			coefficients_->values[5] = dir.z(); //axis_z
			coefficients_->values[6] = radius; //radius

			std::cout << *coefficients_ << std::endl;
		}
		return;
	}

	void Primitive3Cylinder::heuristic_laser_array_cross()
	{
		float cylinder_radius(0.001), cylinder_height(0.001);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

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
			bounds_horizontal.push_back({ bounds_temp[0], bounds_temp[1] });
		}
		std::vector<std::array<float, 2>> bounds_vertical;
		for (int i(0); i < subprims_[1].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[1][i]->getPointCloud()));
			bounds_vertical.push_back({ bounds_temp[2], bounds_temp[3] });
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

			// Find the centroid/center among the points of each subprimitives
			Eigen::Vector3f false_bottom(0.0, 0.0, 0.0), dir(0.0, 0.0, 0.0), v_center(0.0, 0.0, 0.0), h_center(0.0, 0.0, 0.0);
			float v_radius(0.0), h_radius(0.0);
			bool has_line(false);

			// Horizontal primitive
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

				*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
				std::cout << "LINE\n" << std::endl;
			}
#ifndef USE_ELLIPSE
			else if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 7) {
				// Subprimitive is a Circle (cyl coef)
				h_center = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					subprims_[0][horizontal_idx]->getCoefficients()->values[1], //0.0,
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);
				h_radius = subprims_[0][horizontal_idx]->getCoefficients()->values[6];

				*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
				std::cout << "CIRCLE\n" << std::endl;
			}
#else
			else if (subprims_[0][horizontal_idx]->getCoefficients()->values.size() == 11) {
				// Subprimitive is an Ellipse
				h_center = Eigen::Vector3f(
					subprims_[0][horizontal_idx]->getCoefficients()->values[0],
					subprims_[0][horizontal_idx]->getCoefficients()->values[1], //0.0
					subprims_[0][horizontal_idx]->getCoefficients()->values[2]
				);

				// Pick the smallest semi-minor axes (sma) lenght as Radius
				if (subprims_[0][horizontal_idx]->getCoefficients()->values[3] >= subprims_[0][horizontal_idx]->getCoefficients()->values[4]) {
					h_radius = subprims_[0][horizontal_idx]->getCoefficients()->values[4];
				} else {
					h_radius = subprims_[0][horizontal_idx]->getCoefficients()->values[3];
				}

				*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
				std::cout << "ELLIPSE\n" << std::endl;
			}
#endif

			// Vertical primitive
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

				*cloud_suprims += *subprims_[1][vertical_idx]->getPointCloud();
				std::cout << "LINE\n" << std::endl;
			}
#ifndef USE_ELLIPSE
			else if (subprims_[1][vertical_idx]->getCoefficients()->values.size() == 7) {
				// Subprimitive is a Circle (cyl coef)
				v_center = Eigen::Vector3f(
					subprims_[1][vertical_idx]->getCoefficients()->values[0], //0.0,
					subprims_[1][vertical_idx]->getCoefficients()->values[1],
					subprims_[1][vertical_idx]->getCoefficients()->values[2]
				);
				v_radius = subprims_[1][vertical_idx]->getCoefficients()->values[6];

				*cloud_suprims += *subprims_[1][vertical_idx]->getPointCloud();
				std::cout << "CIRCLE\n" << std::endl;
			}
#else
			else if (subprims_[1][vertical_idx]->getCoefficients()->values.size() == 11) {
				// Subprimitive is an Ellipse
				h_center = Eigen::Vector3f(
					subprims_[1][vertical_idx]->getCoefficients()->values[0],
					subprims_[1][vertical_idx]->getCoefficients()->values[1], //0.0
					subprims_[1][vertical_idx]->getCoefficients()->values[2]
				);

				// Pick the smallest semi-minor axes (sma) lenght as Radius
				if (subprims_[1][vertical_idx]->getCoefficients()->values[3] >= subprims_[1][vertical_idx]->getCoefficients()->values[4]) {
					h_radius = subprims_[1][vertical_idx]->getCoefficients()->values[4];
				}else {
					h_radius = subprims_[1][vertical_idx]->getCoefficients()->values[3];
				}

				*cloud_suprims += *subprims_[1][vertical_idx]->getPointCloud();
				std::cout << "ELLIPSE\n" << std::endl;
			}
#endif
			*cloud_ = *cloud_suprims;

			// Find the cylinder bottom center
			Eigen::Vector3f center(0.0, 0.0, 0.0), cyl_bottom_center(0.0, 0.0, 0.0);
			if (!v_center.isZero() != !h_center.isZero()) { //XOR
				if (!v_center.isZero()) {
					center = v_center;
					cylinder_radius = v_radius;					
				}
				else if (!h_center.isZero()) {
					center = h_center;
					cylinder_radius = h_radius;
				}

				Eigen::Vector3f vec(false_bottom.x() - center.x(), false_bottom.y() - center.y(), false_bottom.z() - center.z());
				cyl_bottom_center = Eigen::Vector3f(
					center.x() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.x(),
					center.y() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.y(),
					center.z() + dir.dot(vec) / std::pow(dir.norm(), 2) * dir.z()
				);

				// Cylinder primitive parameters
				cylinder_height = dir.norm();
			}

			// Cylinder coefficients(point_x, point_y, point_z, axis_x, axis_y, axis_z, radius)
			coefficients_->values[0] = cyl_bottom_center.x(); //point_x
			coefficients_->values[1] = cyl_bottom_center.y(); //point_y
			coefficients_->values[2] = cyl_bottom_center.z(); //point_z
			coefficients_->values[3] = dir.x(); //axis_x
			coefficients_->values[4] = dir.y(); //axis_y
			coefficients_->values[5] = dir.z(); //axis_z
			coefficients_->values[6] = cylinder_radius; //radius

			std::cout << *coefficients_ << std::endl;
		}
		return;
	}


	void Primitive3Cylinder::heuristic_laser_array_star()
	{
		// Reset total ellipse count
		ellipse_count_ = 0;
		float sma_diff(0.0f); // ellipses semi-minor axes difference
		
		float radius(0.001);
		Eigen::Vector3f center(0.0, 0.0, 0.0), dir(0.0, 0.0, 0.0);

		// Computing the boundaries of the circle primitives
		//   \ [-] /
		//    \[-]/
		//	 [-----][--------]
		//    /[-]\
		//   / [-] \
		//  /  [-]  \
		
		std::array<pcl::PointXYZ, 4> normals = {
			pcl::PointXYZ(1, 0, 0), // 0 deg
			pcl::PointXYZ(1, 1, 0), // 45 deg
			pcl::PointXYZ(0, 1, 0), // 90 deg
			pcl::PointXYZ(-1, 1, 0) // 135 deg
		};
		// Key-points at the extremes of each subprim crossin the origin will be used by the heuristic
		std::array<pcl::PointXYZ, 8> keypoints;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

		bool has_line(false);
		int idx_line(-1);
		int ellipse_counter(0); // usefull ellipses only
		std::vector<int> ellipse_idx; // usefull ellipses only

		std::array<int, 4> idx({ -1, -1, -1, -1 }); // idx of the selected sub-primitive within each cut (if found)
		std::array<std::array<float, 2>, 4> bounds;
		for (int k(0); k < 4; ++k) {
			// Iterate over the laser cuts
			for (int i(0); i < subprims_[k].size(); ++i) {
				// Iterate over the fitted subprim clouds
				pcl::PointXYZ min, max;
				std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[k][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), normals[k], min, max));
				bounds[i] = { bounds_temp[0], bounds_temp[1] };

				// Find the front face spaned by the intersection '+'				
				if (true) { //if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
					keypoints[k + 4] = min; // ex. negative x-axis -> 4
					keypoints[k] = max; // ex. positive x-axis -> 0					

					if (subprims_[k][i]->getCoefficients()->values.size() == 6 && !has_line) {
						// Subprimitive is a Line
						dir = Eigen::Vector3f(
							subprims_[k][i]->getCoefficients()->values[3],
							subprims_[k][i]->getCoefficients()->values[4],
							subprims_[k][i]->getCoefficients()->values[5]
						);
						has_line = true;
						idx_line = k;

						idx[k] = i;

						*cloud_suprims += *subprims_[k][i]->getPointCloud();
						std::cout << "LINE\n" << std::endl;
					}
					else if (subprims_[k][i]->getCoefficients()->values.size() == 11) { //subprims_[k][i]->getCoefficients()->values[2] > 0.0f //0.050
						// Subprimitive is an Ellipse				
						
						// Pick the smallest semi-minor axes (sma) lenght as the radius
						float ssma(0.0);
						if (subprims_[k][i]->getCoefficients()->values[3] > subprims_[k][i]->getCoefficients()->values[4]) {
							ssma = subprims_[k][i]->getCoefficients()->values[4];
						} else {
							ssma = subprims_[k][i]->getCoefficients()->values[3];
						}

						sma_diff += std::abs(subprims_[k][i]->getCoefficients()->values[3] - subprims_[k][i]->getCoefficients()->values[4]);

						// Exclude the larger ellipse(s) for center and radius calculation if the radius ssma > 0.05
						if (ssma < 0.05) {							
							center += Eigen::Vector3f(
								subprims_[k][i]->getCoefficients()->values[0],
								subprims_[k][i]->getCoefficients()->values[1],
								subprims_[k][i]->getCoefficients()->values[2]
							);
							radius += ssma;

							ellipse_idx.push_back(k);
							ellipse_counter += 1;
						}

						idx[k] = i;

						*cloud_suprims += *subprims_[k][i]->getPointCloud();
						++ellipse_count_;
						std::cout << "ELLIPSE\n" << std::endl;
					}
				}
			}
		}
		*cloud_ = *cloud_suprims;

		// Update the cylinder vars that depend on ellipses
		if (ellipse_counter >= 0) {
			radius /= static_cast<float>(ellipse_counter);
			center /= static_cast<float>(ellipse_counter);
		}

		// Checks the minimal number (three) of ellipse cuts
		if (ellipse_count_ < 3) {
			return;
		}
		// Checks if all ellipses approximate a sphere instead
		if (ellipse_count_ == 4 && sma_diff < 0.025) {
			return;
		}

		if (subprims_.size() == 4 && idx[0] != -1 && idx[1] != -1 && idx[2] != -1 && idx[3] != -1) {
			std::cout << "idx_0: " << idx[0] << " idx_1: " << idx[1] << " idx_2: " << idx[2] << " idx_3: " << idx[3] << std::endl;

			if (!has_line) {
				// Find the plane that contains the extreme keypoints of the ellipses
				std::array<Eigen::Vector3f, 4> plane_pts; // (four keypoints at most)
				int p_counter(0);
				for (auto k : ellipse_idx) {
					// Alternate the selection of min or max points
					if (k % 2 == 0) {
						plane_pts[p_counter] = Eigen::Vector3f(keypoints[k].x, keypoints[k].y, keypoints[k].z);
						++p_counter;
					}
					else {
						plane_pts[p_counter] = Eigen::Vector3f(keypoints[k + 4].x, keypoints[k + 4].y, keypoints[k + 4].z);
						++p_counter;
					}
				}
				Eigen::Vector3f plane_normal = (plane_pts[1] - plane_pts[0]).cross((plane_pts[2] - plane_pts[1])).normalized();

				// Project all ellipses keypoints onto the plane
				for (size_t i(0); i < keypoints.size(); ++i) {
					Eigen::Vector3f kp(keypoints[i].x, keypoints[i].y, keypoints[i].z), kp_proj(0.0, 0.0, 0.0);
					kp_proj = kp - (kp - center).dot(plane_normal) * plane_normal / plane_normal.squaredNorm();
					keypoints[i] = pcl::PointXYZ(kp_proj(0), kp_proj(1), kp_proj(2));
				}

				// Longitudinal "direction" vector to be found by the heuristic
				// (this orientation heuristic is based on the projected keypoints)
				for (size_t i(0); i < keypoints.size(); ++i) {
					Eigen::Vector3f kp(0.0, 0.0, 0.0), kp_prev(0.0, 0.0, 0.0), kp_next(0.0, 0.0, 0.0);
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

					// Estimate the direction
					float ang_threshold(10.0f); //5.0
					if (angle < ang_threshold * M_PI / 180.0) {
						dir = kp_next - kp_prev;
						dir.normalize();
						std::cout << "Angle between vectors: " << angle * 180.0 / M_PI << std::endl;
						break;
					}
				}
			}

			// Cylinder coefficients(point_x, point_y, point_z, axis_x, axis_y, axis_z, radius)
			coefficients_->values[0] = center.x(); //point_x
			coefficients_->values[1] = center.y(); //point_y
			coefficients_->values[2] = center.z(); //point_z
			coefficients_->values[3] = dir.x(); //axis_x
			coefficients_->values[4] = dir.y(); //axis_y
			coefficients_->values[5] = dir.z(); //axis_z
			coefficients_->values[6] = radius; //radius
			std::cout << *coefficients_ << std::endl;			
			std::cout << "Ellipse count: " << ellipse_count_ << '\n';
			std::cout << "Subprims count: " << subprims_.size() << '\n';
		}
		return;
	}

	/* Checks if the heuristic is valid. */
	bool Primitive3Cylinder::is_heuristic_valid()
	{
		if (!cloud_->points.empty()) {
			/* x_min, x_max, y_min, y_max, z_min, z_max. */
			std::array<float, 6> ranges(getPointCloudRanges(*cloud_));
			/* Checks the z-coordinate of the Primitive3Cylinder center. */
			if (!(coefficients_->values[2] > ranges[4])) {
				this->reset();
				return false;
			}
		} else {
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
		} else {
			this->reset();
			return false;
		}

		/* Checks if the components of the direction vector and the radius are valid. */
		if (!(std::sqrt(
			coefficients_->values[3] * coefficients_->values[3] +
			coefficients_->values[4] * coefficients_->values[4] +
			coefficients_->values[5] * coefficients_->values[5]) > 0 &&
			coefficients_->values[6] > 0))
		{
			this->reset();
			return false;
		}		

		isempty_ = false;
		return true;
	}

}