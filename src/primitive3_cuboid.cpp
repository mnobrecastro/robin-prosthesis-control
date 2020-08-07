#include "primitive3_cuboid.h"

namespace robin
{
	Primitive3Cuboid::Primitive3Cuboid()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 10-element cube coefficients
		// {Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth}
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
		coefficients_->values.push_back(0.0); //4
		coefficients_->values.push_back(0.0); //5
		coefficients_->values.push_back(0.0); //6
		coefficients_->values.push_back(0.0); //7
		coefficients_->values.push_back(0.0); //8
		coefficients_->values.push_back(0.0); //9
	}
	Primitive3Cuboid::~Primitive3Cuboid() {}

	void Primitive3Cuboid::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		//if (visualizeOnOff_)
		viewer->addCube(*coefficients_, "cube");
		for (auto plane : planes_) {
			plane->setVisualizeOnOff(false);
			plane->visualize(viewer);
		}
	}

	void Primitive3Cuboid::reset()
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
		if (planes_.size() > 0) {
			return true;
			std::cout << "FLAG";
		}
		this->reset();
		return false;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Cuboid::correct_coefficients()
	{
		float width(0.0), height(0.0), depth(0.001);
		Eigen::Matrix3f mori;
		Eigen::Vector3f face_center, cube_center;
		Eigen::Vector3f intersection_axis, point_on_axis;

		if (planes_.size() == 1) {
			width = planes_[0]->getProperty_width();
			height = planes_[0]->getProperty_height();


			mori(0, 0) = planes_[0]->getProperty_e0_x();
			mori(1, 0) = planes_[0]->getProperty_e0_y();
			mori(2, 0) = planes_[0]->getProperty_e0_z();

			mori(0, 1) = planes_[0]->getProperty_e1_x();
			mori(1, 1) = planes_[0]->getProperty_e1_y();
			mori(2, 1) = planes_[0]->getProperty_e1_z();

			mori(0, 2) = planes_[0]->getProperty_axis_x();
			mori(1, 2) = planes_[0]->getProperty_axis_y();
			mori(2, 2) = planes_[0]->getProperty_axis_z();

			face_center(0) = planes_[0]->getProperty_center_x();
			face_center(1) = planes_[0]->getProperty_center_y();
			face_center(2) = planes_[0]->getProperty_center_z();

			cube_center(0) = face_center.x() + mori(0, 2) * depth/2;
			cube_center(1) = face_center.y() + mori(1, 2) * depth/2;
			cube_center(2) = face_center.z() + mori(2, 2) * depth/2;

		} else {
			// Case of 2 or 3
			float a1(planes_[0]->getProperty_axis_x());
			float b1(planes_[0]->getProperty_axis_y());
			float c1(planes_[0]->getProperty_axis_z());
			float d1(planes_[0]->getProperty_d());

			float a2(planes_[1]->getProperty_axis_x());
			float b2(planes_[1]->getProperty_axis_y());
			float c2(planes_[1]->getProperty_axis_z());
			float d2(planes_[1]->getProperty_d());

			// Calculate the intersection axis between the two planes
			intersection_axis(0) = b1/a1 * (c2-a2*c1/a1) / (b2-a2*b1/a1) - c1 / a1;
			intersection_axis(1) = -(c2-a2*c1/a1) / (b2-a2*b1/a1);
			intersection_axis(2) = 1;

			// Calculate a point on the axis
			point_on_axis(0) = intersection_axis(0) + b1/a1 * (d2-a2*d1/a1) / (b2-a2*b1/a1) - d1/a1;
			point_on_axis(1) = intersection_axis(1) - (d2-a2*d1/a1) / (b2-a2*b1/a1);
			point_on_axis(2) = 0;

			// Calculate height from projecting all points from all planes
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto plane : planes_) {
				*cloud += *plane->getPointCloud();
			}
			std::array<float, 2> height_arr(getPointCloudExtremes(
				*cloud,
				pcl::PointXYZ(point_on_axis.x(),point_on_axis.y(),point_on_axis.z()),
				pcl::PointXYZ(intersection_axis.x(),intersection_axis.y(),intersection_axis.z())
			));
			height = height_arr[1]- height_arr[0];

			// Calculate width
			Eigen::Vector3f face0_axis = { planes_[0]->getProperty_axis_x(), planes_[0]->getProperty_axis_y(), planes_[0]->getProperty_axis_z() };
			Eigen::Vector3f face0_axis_new = intersection_axis.cross(face0_axis);
			std::array<float, 2> width_arr(getPointCloudExtremes(
				*planes_[0]->getPointCloud(),
				pcl::PointXYZ(planes_[0]->getProperty_center_x(), planes_[0]->getProperty_center_y(), planes_[0]->getProperty_center_z()),
				pcl::PointXYZ(intersection_axis.x(), intersection_axis.y(), intersection_axis.z())
			));
			width = width_arr[1] - width_arr[0];

			// Calculate depth
			Eigen::Vector3f face1_axis = { planes_[1]->getProperty_axis_x(), planes_[1]->getProperty_axis_y(), planes_[1]->getProperty_axis_z() };
			Eigen::Vector3f face1_axis_new = intersection_axis.cross(face1_axis);
			std::array<float, 2> depth_arr(getPointCloudExtremes(
				*planes_[1]->getPointCloud(),
				pcl::PointXYZ(planes_[1]->getProperty_center_x(), planes_[1]->getProperty_center_y(), planes_[1]->getProperty_center_z()),
				pcl::PointXYZ(intersection_axis.x(), intersection_axis.y(), intersection_axis.z())
			));
			depth = depth_arr[1] - depth_arr[0];


			mori(0, 0) = face0_axis_new.x();
			mori(1, 0) = face0_axis_new.y();
			mori(2, 0) = face0_axis_new.z();

			mori(0, 1) = intersection_axis.x();
			mori(1, 1) = intersection_axis.y();
			mori(2, 1) = intersection_axis.z();

			mori(0, 2) = planes_[0]->getProperty_axis_x();
			mori(1, 2) = planes_[0]->getProperty_axis_y();
			mori(2, 2) = planes_[0]->getProperty_axis_z();

			face_center(0) = planes_[0]->getProperty_center_x();
			face_center(1) = planes_[0]->getProperty_center_y();
			face_center(2) = planes_[0]->getProperty_center_z();

			cube_center(0) = face_center.x() + mori(0, 2) * depth / 2;
			cube_center(1) = face_center.y() + mori(1, 2) * depth / 2;
			cube_center(2) = face_center.z() + mori(2, 2) * depth / 2;
		}

		Eigen::Quaternionf quat(mori);

		//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
		coefficients_->values[0] = cube_center.x(); //Tx
		coefficients_->values[1] = cube_center.y(); //Ty
		coefficients_->values[2] = cube_center.z(); //Tz
		coefficients_->values[3] = quat.x(); //Qx
		coefficients_->values[4] = quat.y(); //Qy
		coefficients_->values[5] = quat.z(); //Qz
		coefficients_->values[6] = quat.w(); //Qw
		coefficients_->values[7] = width;
		coefficients_->values[8] = height;
		coefficients_->values[9] = depth;
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Cuboid::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.width = coefficients_->values[7];
		properties_.height = coefficients_->values[8];
		properties_.depth = coefficients_->values[9];
	}
}