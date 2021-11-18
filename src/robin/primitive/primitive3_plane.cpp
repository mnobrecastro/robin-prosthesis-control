#include "primitive3_plane.h"

namespace robin
{
	Primitive3Plane::Primitive3Plane()
		: Primitive3Plane(PLANE_TYPE::DEFAULT, Eigen::Vector3f(0.0,0.0,0.0), 0.0)
	{}
	
	/* Primitive3Plane can be initialized by "DEFAULT" or by "PERPENDICULAR" or "PARALLEL" types. */
	Primitive3Plane::Primitive3Plane(PLANE_TYPE type, Eigen::Vector3f v, float angle)
	{		
		type_ = type;
		properties_.v = v;
		properties_.angle = angle;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 4-element plane coefficients
		// {normal_x, normal_y, normal_z, d}
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
	}
	Primitive3Plane::~Primitive3Plane() {}
	
	void Primitive3Plane::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = 0.000;
		coefficients_->values[1] = 0.000;
		coefficients_->values[2] = 0.000;
		coefficients_->values[3] = 0.000;
	}

	void Primitive3Plane::setCoefficients(std::vector<float> v)
	{
		if (v.size() == 4) {
			coefficients_->values[0] = v[0];
			coefficients_->values[1] = v[1];
			coefficients_->values[2] = v[2];
			coefficients_->values[3] = v[3];

			isempty_ = false;
		}
	}
	
	void Primitive3Plane::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		if (visualizeOnOff_) {
			//Render the Primitive3Plane as cube object with small thickness

			pcl::ModelCoefficients::Ptr coefs_cube(new pcl::ModelCoefficients);
			// Initialization of the 10-element cube coefficients
			// {Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth}
			coefs_cube->values.push_back(0.0); //0
			coefs_cube->values.push_back(0.0); //1
			coefs_cube->values.push_back(0.0); //2
			coefs_cube->values.push_back(0.0); //3
			coefs_cube->values.push_back(0.0); //4
			coefs_cube->values.push_back(0.0); //5
			coefs_cube->values.push_back(0.0); //6
			coefs_cube->values.push_back(0.0); //7
			coefs_cube->values.push_back(0.0); //8
			coefs_cube->values.push_back(0.0); //9

			float width(0.0), height(0.0), depth(0.001);
			Eigen::Vector3f e0_axis, e1_axis, z_axis;
			Eigen::Matrix3f mori;
			Eigen::Vector3f face_center, cube_center;
			
			/*
			 *    ___________
			 *   |   Z ^    ||
			 *   |     |    ||
			 *   |     |    ||
			 *   |     ---> ||
			 *   |    /     ||
			 *   |  e0	    ||
			 *   |__________/
			 */

			width = properties_.width; // In the e1-direction
			height = properties_.height; // In the z-direction

			// Plane z -> Cube e0
			e0_axis = { properties_.axis_x, properties_.axis_y, properties_.axis_z };
			e0_axis.normalize();
			e0_axis *= depth / 2;
			// Plane -e1 -> Cube e1
			e1_axis = { -properties_.e1_x, -properties_.e1_y, -properties_.e1_z };
			e1_axis.normalize();
			e1_axis *= width / 2;
			// Plane e0 -> Cube z
			z_axis = { properties_.e0_x, properties_.e0_y, properties_.e0_z };
			z_axis.normalize();
			z_axis *= height / 2;
			
			face_center(0) = properties_.center_x;
			face_center(1) = properties_.center_y;
			face_center(2) = properties_.center_z;

			cube_center(0) = face_center.x() - e0_axis(0);
			cube_center(1) = face_center.y() - e0_axis(1);
			cube_center(2) = face_center.z() - e0_axis(2);

			// Transformation matrix
			e0_axis.normalize();
			mori(0, 0) = e0_axis(0);
			mori(1, 0) = e0_axis(1);
			mori(2, 0) = e0_axis(2);
			e1_axis.normalize();
			mori(0, 1) = e1_axis(0);
			mori(1, 1) = e1_axis(1);
			mori(2, 1) = e1_axis(2);
			z_axis.normalize();
			mori(0, 2) = z_axis(0);
			mori(1, 2) = z_axis(1);
			mori(2, 2) = z_axis(2);

			Eigen::Quaternionf quat(mori);

			//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
			coefs_cube->values[0] = cube_center.x(); //Tx
			coefs_cube->values[1] = cube_center.y(); //Ty
			coefs_cube->values[2] = cube_center.z(); //Tz
			coefs_cube->values[3] = quat.x(); //Qx
			coefs_cube->values[4] = quat.y(); //Qy
			coefs_cube->values[5] = quat.z(); //Qz
			coefs_cube->values[6] = quat.w(); //Qw
			coefs_cube->values[7] = depth; //width;
			coefs_cube->values[8] = width; //height;
			coefs_cube->values[9] = height; //depth;

			viewer->addCube(*coefs_cube, "plane" + std::to_string(std::rand()));
		}

		pcl::PointXYZ center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ center_normal(properties_.center_x + 0.05 * properties_.axis_x,
									properties_.center_y + 0.05 * properties_.axis_y,
									properties_.center_z + 0.05 * properties_.axis_z);
		viewer->addLine(center, center_normal, "plane_normal" + std::to_string(std::rand()));
		pcl::PointXYZ center_e0(properties_.center_x + properties_.e0_x,
								properties_.center_y + properties_.e0_y,
								properties_.center_z + properties_.e0_z);
		viewer->addLine(center, center_e0, "plane_e0" + std::to_string(std::rand()));
		pcl::PointXYZ center_e1(properties_.center_x + properties_.e1_x,
								properties_.center_y + properties_.e1_y,
								properties_.center_z + properties_.e1_z);
		viewer->addLine(center, center_e1, "plane_e1" + std::to_string(std::rand()));
	}



	void Primitive3Plane::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			switch (type_)
			{
			case PLANE_TYPE::DEFAULT:
				fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE);
				break;
			case PLANE_TYPE::PERPENDICULAR:
				/* pcl::SACMODEL_NORMAL_PERPENDICULAR_PLANE
				 * NOT IMPLEMENTED IN THE POINT CLOUD LIBRARY!
				 * pcl\segmentation\impl\sac_segmentation.hpp */
				std::cerr << "The 'pcl::SACMODEL_NORMAL_PERPENDICULAR_PLANE' is not available. Using 'pcl::SACMODEL_NORMAL_PLANE' by default." << std::endl;
				fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE);
				break;
			case PLANE_TYPE::PARALLEL:
				fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
				break;
			}			
		}
		else {
			switch (type_)
			{
			case PLANE_TYPE::DEFAULT:
				fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_PLANE);
				break;
			case PLANE_TYPE::PERPENDICULAR:
				fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_PERPENDICULAR_PLANE);
				break;
			case PLANE_TYPE::PARALLEL:
				fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_PARALLEL_PLANE);
				break;
			}
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
			switch (type_)
			{
			case PLANE_TYPE::DEFAULT:
				seg->setModelType(pcl::SACMODEL_NORMAL_PLANE);
				fit_sample_consensus_with_normals(cloud, seg);
				break;
			case PLANE_TYPE::PERPENDICULAR:
				/* pcl::SACMODEL_NORMAL_PERPENDICULAR_PLANE
				 * NOT IMPLEMENTED IN THE POINT CLOUD LIBRARY!
				 * pcl\segmentation\impl\sac_segmentation.hpp */
				std::cerr << "The 'pcl::SACMODEL_NORMAL_PERPENDICULAR_PLANE' is not available. Using 'pcl::SACMODEL_NORMAL_PLANE' by default." << std::endl;
				seg->setModelType(pcl::SACMODEL_NORMAL_PLANE);
				fit_sample_consensus_with_normals(cloud, seg);
				break;
			case PLANE_TYPE::PARALLEL:
				seg->setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
				fit_sample_consensus_with_normals(cloud, seg);
				break;
			}
		}
		else {
			switch (type_)
			{
			case PLANE_TYPE::DEFAULT:
				seg->setModelType(pcl::SACMODEL_PLANE);
				fit_sample_consensus(cloud, seg);
				break;
			case PLANE_TYPE::PERPENDICULAR:
				seg->setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
				fit_sample_consensus(cloud, seg);
				break;
			case PLANE_TYPE::PARALLEL:
				seg->setModelType(pcl::SACMODEL_PARALLEL_PLANE);
				fit_sample_consensus(cloud, seg);
				break;
			}
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
		if (!cloud_->points.empty()) {
			return true;
		}

		this->reset();
		return false;
	}

	/* Correct the obtained coefficients if necessary. */
	void Primitive3Plane::correct_coefficients()
	{
		// Normal always points towards the camera -Z (assumes objects are convex)
		if (coefficients_->values[2] > 0) {
			coefficients_->values[0] = -coefficients_->values[0];
			coefficients_->values[1] = -coefficients_->values[1];
			coefficients_->values[2] = -coefficients_->values[2];
		}
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Plane::update_properties()
	{
		Eigen::Vector3f e2(coefficients_->values[0], coefficients_->values[1], coefficients_->values[2]);
		
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

		Eigen::Vector3f e1 = e2.cross(e0);
		e1.normalize();
		std::array<float, 2> e1_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(e1.x(), e1.y(), e1.z()))
		);

		properties_.center_x = mean.x();
		properties_.center_y = mean.y();
		properties_.center_z = mean.z();
		properties_.axis_x = coefficients_->values[0];
		properties_.axis_y = coefficients_->values[1];
		properties_.axis_z = coefficients_->values[2];
		properties_.d = coefficients_->values[3];
		properties_.e0_x = e0.x() * (e0_min_max[1]-e0_min_max[0])/2;
		properties_.e0_y = e0.y() * (e0_min_max[1]-e0_min_max[0])/2;
		properties_.e0_z = e0.z() * (e0_min_max[1]-e0_min_max[0])/2;
		properties_.e1_x = e1.x() * (e1_min_max[1]-e1_min_max[0])/2;
		properties_.e1_y = e1.y() * (e1_min_max[1]-e1_min_max[0])/2;
		properties_.e1_z = e1.z() * (e1_min_max[1]-e1_min_max[0])/2;
		properties_.height = e0_min_max[1] - e0_min_max[0]; // e0 sets convention for HEIGHT
		properties_.width = e1_min_max[1] - e1_min_max[0]; // e1 sets convention for WIDTH
	}
}