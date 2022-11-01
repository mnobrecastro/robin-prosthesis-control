#include "primitive3.h"

namespace robin
{
	Primitive3::Primitive3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
	}
	Primitive3::~Primitive3() {}

	Primitive3::Primitive3(const Primitive3& prim)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*prim.getPointCloud()));
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients(*prim.getCoefficients()));
		coefficients_ = coefficients;
				
		properties_.center_x = prim.properties_.center_x;
		properties_.center_y = prim.properties_.center_y;
		properties_.center_z = prim.properties_.center_z;
		properties_.width = prim.properties_.width;
		properties_.height = prim.properties_.height;
		properties_.depth = prim.properties_.depth;
		properties_.axis_x = prim.properties_.axis_x;
		properties_.axis_y = prim.properties_.axis_y;
		properties_.axis_z = prim.properties_.axis_z;
		properties_.d = prim.properties_.d;
		properties_.radius = prim.properties_.radius;
		properties_.e0_x = prim.properties_.e0_x;
		properties_.e0_y = prim.properties_.e0_y;
		properties_.e0_z = prim.properties_.e0_z;
		properties_.e1_x = prim.properties_.e1_x;
		properties_.e1_y = prim.properties_.e1_y;
		properties_.e1_z = prim.properties_.e1_z;
	}

	void Primitive3::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		pcl::PointXYZ center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ center_normal(properties_.center_x + 0.05 * properties_.axis_x,
			properties_.center_y + 0.05 * properties_.axis_y,
			properties_.center_z + 0.05 * properties_.axis_z);
		viewer->addLine(center, center_normal, "pca_ez" + std::to_string(std::rand()));
		pcl::PointXYZ center_e0(properties_.center_x + properties_.e0_x,
			properties_.center_y + properties_.e0_y,
			properties_.center_z + properties_.e0_z);
		viewer->addLine(center, center_e0, "pca_e0" + std::to_string(std::rand()));
		pcl::PointXYZ center_e1(properties_.center_x + properties_.e1_x,
			properties_.center_y + properties_.e1_y,
			properties_.center_z + properties_.e1_z);
		viewer->addLine(center, center_e1, "pca_e1" + std::to_string(std::rand()));
	}



	void Primitive3::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptc(new pcl::PointCloud<pcl::PointXYZ>(cloud));
		cloud_ = ptc;
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Primitive3::getPointCloud() const
	{
		return cloud_;
	}

	pcl::ModelCoefficients::Ptr Primitive3::getCoefficients() const
	{
		return coefficients_;
	}

	void Primitive3::reset()
	{
		cloud_->clear();
		coefficients_->values.clear();		
		this->update_properties();
		return;
	}

	void Primitive3::pca(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptc(new pcl::PointCloud<pcl::PointXYZ>(cloud));
		cloud_ = ptc;

		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(cloud_);
		Eigen::Vector4f mean(pca.getMean());
		Eigen::Matrix3f eigenvecs(pca.getEigenVectors());
		Eigen::Vector3f eigenvals(pca.getEigenValues());
		//Eigen::MatrixXf coeffs(pca.getCoefficients());

		Eigen::Vector3f ez = eigenvecs.col(0);
		ez.normalize();
		std::array<float, 2> ez_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(ez.x(), ez.y(), ez.z()))
		);

		Eigen::Vector3f e0 = eigenvecs.col(1);
		e0.normalize();
		std::array<float, 2> e0_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(e0.x(), e0.y(), e0.z()))
		);

		Eigen::Vector3f e1 = eigenvecs.col(2);
		e1.normalize();
		std::array<float, 2> e1_min_max(
			getPointCloudExtremes(*cloud_, pcl::PointXYZ(mean.x(), mean.y(), mean.z()), pcl::PointXYZ(e1.x(), e1.y(), e1.z()))
		);

		properties_.center_x = mean.x();
		properties_.center_y = mean.y();
		properties_.center_z = mean.z();
		properties_.axis_x = ez.x() * (ez_min_max[1] - ez_min_max[0]) / 2;
		properties_.axis_y = ez.y() * (ez_min_max[1] - ez_min_max[0]) / 2;
		properties_.axis_z = ez.z() * (ez_min_max[1] - ez_min_max[0]) / 2;
		properties_.e0_x = e0.x() * (e0_min_max[1] - e0_min_max[0]) / 2;
		properties_.e0_y = e0.y() * (e0_min_max[1] - e0_min_max[0]) / 2;
		properties_.e0_z = e0.z() * (e0_min_max[1] - e0_min_max[0]) / 2;
		properties_.e1_x = e1.x() * (e1_min_max[1] - e1_min_max[0]) / 2;
		properties_.e1_y = e1.y() * (e1_min_max[1] - e1_min_max[0]) / 2;
		properties_.e1_z = e1.z() * (e1_min_max[1] - e1_min_max[0]) / 2;
		properties_.depth = ez_min_max[1] - ez_min_max[0]; // ez sets convention for DEPTH
		properties_.height = e0_min_max[1] - e0_min_max[0]; // e0 sets convention for HEIGHT
		properties_.width = e1_min_max[1] - e1_min_max[0]; // e1 sets convention for WIDTH
		return;
	}


	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		std::cout << "Using default instance of 'sample consensus'.\n";
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ>* seg;
		seg->setOptimizeCoefficients(true);
		seg->setModelType(SAC_MODEL);
		seg->setMethodType(SAC_METHOD);
		seg->setMaxIterations(100);
		seg->setDistanceThreshold(0.0025);
		if (SAC_MODEL == pcl::SACMODEL_CYLINDER || SAC_MODEL == pcl::SACMODEL_SPHERE) {
			seg->setRadiusLimits(0.005, 0.050);
		}
		if (SAC_MODEL == pcl::SACMODEL_PERPENDICULAR_PLANE || SAC_MODEL == pcl::SACMODEL_PARALLEL_PLANE) {
			seg->setAxis(properties_.v);
			seg->setEpsAngle(properties_.angle * M_PI/180);
		}

		this-> fit_sample_consensus(cloud, seg);
		return;
	}

	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		// Save a copy of the coefficients in case a solution is not found
		pcl::ModelCoefficients::Ptr coef_temp_copy(new pcl::ModelCoefficients(*coefficients_));
		
		std::cout << "Running 'sample consensus'...";

		// Copy the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(seg_obj->getOptimizeCoefficients());
		seg.setModelType(seg_obj->getModelType());
		seg.setMethodType(seg_obj->getMethodType());
		seg.setMaxIterations(seg_obj->getMaxIterations());
		seg.setDistanceThreshold(seg_obj->getDistanceThreshold());
		double min_radius, max_radius;
		seg_obj->getRadiusLimits(min_radius, max_radius);
		seg.setRadiusLimits(min_radius, max_radius); // pcl::SACMODEL_CYLINDER || pcl::SACMODEL_SPHERE
		seg.setAxis(properties_.v); // pcl::SACMODEL_PERPENDICULAR_PLANE || pcl::SACMODEL_PARALLEL_PLANE
		seg.setEpsAngle(properties_.angle * M_PI/180); // pcl::SACMODEL_PERPENDICULAR_PLANE || pcl::SACMODEL_PARALLEL_PLANE
		seg.setInputCloud(cloud);

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);

		if (!coefficients_->values.empty()) {
			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_);

			// Remove the inliers, extract/subtract the rest
			extract.setNegative(true);
			extract.filter(*cloud);

			std::cout << " done.\n";
		}
		else {
			coefficients_ = coef_temp_copy;
			std::cout << " no solution found.\n";
		}
		return;
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		std::cout << "Using default instance of 'sample consensus from normals'.\n";
		// Create the segmentation object
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>* seg;
		seg->setOptimizeCoefficients(true);
		seg->setModelType(SAC_MODEL);
		seg->setNormalDistanceWeight(0.1);
		seg->setMethodType(SAC_METHOD);
		seg->setMaxIterations(100);
		seg->setDistanceThreshold(0.0025);
		if (SAC_MODEL == pcl::SACMODEL_CYLINDER || SAC_MODEL == pcl::SACMODEL_SPHERE) {
			seg->setRadiusLimits(0.005, 0.050);
		}
		if (SAC_MODEL == pcl::SACMODEL_PERPENDICULAR_PLANE || SAC_MODEL == pcl::SACMODEL_PARALLEL_PLANE) {
			seg->setAxis(properties_.v);
			seg->setEpsAngle(properties_.angle * M_PI/180);
		}

		this->fit_sample_consensus_with_normals(cloud, seg);
		return;
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		// Save a copy of the coefficients in case a solution is not found
		pcl::ModelCoefficients::Ptr coef_temp_copy(new pcl::ModelCoefficients(*coefficients_));
		
		std::cout << "Running 'sample consensus from normals'...";

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		// Estimate point normals
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;		
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);		

		// Copy the segmentation object
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setNormalDistanceWeight(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getNormalDistanceWeight());
		seg.setOptimizeCoefficients(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getOptimizeCoefficients());
		seg.setModelType(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getModelType());
		seg.setMethodType(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getMethodType());
		seg.setMaxIterations(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getMaxIterations());
		seg.setDistanceThreshold(static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getDistanceThreshold());
		double min_radius, max_radius;
		static_cast<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>*>(seg_obj)->getRadiusLimits(min_radius, max_radius);
		seg.setRadiusLimits(min_radius, max_radius);
		seg.setAxis(properties_.v); // pcl::SACMODEL_PERPENDICULAR_PLANE || pcl::SACMODEL_PARALLEL_PLANE
		seg.setEpsAngle(properties_.angle * M_PI/180); // pcl::SACMODEL_PERPENDICULAR_PLANE || pcl::SACMODEL_PARALLEL_PLANE
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals);

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);

		if (!coefficients_->values.empty()) {
			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_);

			// Remove the inliers, extract/subtract the rest
			extract.setNegative(true);
			extract.filter(*cloud);

			std::cout << " done.\n";
		}
		else {
			coefficients_ = coef_temp_copy;
			std::cout << " no solution found.\n";
		}
		return;
	}


	void Primitive3::setVisualizeOnOff(bool visual)
	{
		visualizeOnOff_ = visual;
		return;
	}



	//// PRIMITIVE3D3 ////

	void Primitive3d3::addSubPrimitive(Primitive3d1* p)
	{
		std::vector<Primitive3d1*> arr;
		arr.push_back(p);
		subprims_.push_back(arr);
		are_subprims_custom_ = true;
		return;
	}

	void Primitive3d3::heuristic(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr, pcl::SACSegmentation<pcl::PointXYZ>* seg, HEURISTIC heu)
	{
		if (!cloud_->points.empty()) {
			cloud_->clear();
		}
		if (!subprims_.empty()) {
			for (auto arr : subprims_) {
				for (auto sp : arr) {
					delete sp;
				}
				arr.clear();
			}
			subprims_.clear();
		}

		for (auto cloud : cloud_arr) {
			if (!are_subprims_custom_) {
				this->cut(cloud, seg);
			}
		}		
		
		if (!this->heuristic_check(heu)) {
			std::cout << "The number of sub-primitives (" << subprims_.size() << " found) does not match the Heuristic.\n";
			return;
		} //(redundant?)

		this->heuristic_prim(heu);

		/* 1. Validate the heuristic. */
		if (this->is_heuristic_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();

		return;
	}

	bool Primitive3d3::heuristic_check(HEURISTIC heu)
	{
		size_t heu_size;
		switch (heu) {
		case robin::HEURISTIC::LASER_ARRAY_SINGLE:
			heu_size = LASER_ARRAY_SINGLE_SIZE;
			break;
		case robin::HEURISTIC::LASER_ARRAY_CROSS:
			heu_size = LASER_ARRAY_CROSS_SIZE;
			break;
		case robin::HEURISTIC::LASER_ARRAY_STAR:
			heu_size = LASER_ARRAY_STAR_SIZE;
			break;
		}
		if (subprims_.size() == heu_size) {
			return true;
		} else {
			return false;
		}
	}

	void Primitive3d3::heuristic_prim(HEURISTIC heu)
	{
		switch (heu) {
		case robin::HEURISTIC::LASER_ARRAY_SINGLE:
			this->heuristic_laser_array_single();
			break;
		case robin::HEURISTIC::LASER_ARRAY_CROSS:
			this->heuristic_laser_array_cross();
			break;
		case robin::HEURISTIC::LASER_ARRAY_STAR:
			this->heuristic_laser_array_star();
			break;
		}
		return;
	}



	//// POINTCLOUD UTILS ////

	float Primitive3::dotPointXYZ(pcl::PointXYZ a, pcl::PointXYZ b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	float Primitive3::normPointXYZ(pcl::PointXYZ c)
	{
		return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
	}

	std::array<float, 2> Primitive3::getPointCloudExtremes(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ center, pcl::PointXYZ axis)
	{
		std::array<float, 2> arr = { 1000.0, -1000.0 };
		pcl::PointXYZ vec;
		float scalar_proj;
		for (size_t i = 0; i < cloud.points.size(); ++i) {
			vec.x = cloud.points[i].x - center.x;
			vec.y = cloud.points[i].y - center.y;
			vec.z = cloud.points[i].z - center.z;
			scalar_proj = dotPointXYZ(axis, vec) / normPointXYZ(axis);
			if (scalar_proj < arr[0])
				arr[0] = scalar_proj;
			if (scalar_proj > arr[1])
				arr[1] = scalar_proj;
		}
		return arr;
	}

	std::array<float, 2> Primitive3::getPointCloudExtremes(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ center, pcl::PointXYZ axis, pcl::PointXYZ& min, pcl::PointXYZ& max)
	{
		std::array<float, 2> arr = { 1000.0, -1000.0 };
		pcl::PointXYZ vec;
		float scalar_proj;
		for (size_t i = 0; i < cloud.points.size(); ++i) {
			vec.x = cloud.points[i].x - center.x;
			vec.y = cloud.points[i].y - center.y;
			vec.z = cloud.points[i].z - center.z;
			scalar_proj = dotPointXYZ(axis, vec) / normPointXYZ(axis);
			if (scalar_proj < arr[0]){
				arr[0] = scalar_proj;
				min.x = cloud.points[i].x;
				min.y = cloud.points[i].y;
				min.z = cloud.points[i].z;
			}
			if (scalar_proj > arr[1]) {
				arr[1] = scalar_proj;
				max.x = cloud.points[i].x;
				max.y = cloud.points[i].y;
				max.z = cloud.points[i].z;
			}
		}
		return arr;
	}

	std::array<float, 6> Primitive3::getPointCloudRanges(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		std::array<float, 6> arr = { 1000.0, -1000.0, 1000.0, -1000.0, 1000.0, -1000.0 };
		for (auto point : cloud.points) {
			if (point.x < arr[0])
				arr[0] = point.x;
			if (point.x > arr[1])
				arr[1] = point.x;
			if (point.y < arr[2])
				arr[2] = point.y;
			if (point.y > arr[3])
				arr[3] = point.y;
			if (point.z < arr[4])
				arr[4] = point.z;
			if (point.z > arr[5])
				arr[5] = point.z;
		}
		return arr;
	}

}