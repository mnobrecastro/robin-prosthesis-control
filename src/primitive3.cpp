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
	}

	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		std::cout << "Using default instance of 'sample consensus'." << std::endl;
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

		this-> fit_sample_consensus(cloud, seg);
	}

	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
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
		seg.setRadiusLimits(min_radius, max_radius);
		seg.setInputCloud(cloud);

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_);

		// Remove the inliers, extract/subtract the rest
		extract.setNegative(true);
		extract.filter(*cloud);

		std::cout << " done." << std::endl;
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		std::cout << "Using default instance of 'sample consensus from normals'." << std::endl;
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

		this->fit_sample_consensus_with_normals(cloud, seg);
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
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
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals);
		//if (seg.getModelType() == pcl::SACMODEL_CYLINDER || seg.getModelType() == pcl::SACMODEL_SPHERE) {
		//seg.setRadiusLimits(0.005, 0.050);
		//}

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_);

		// Remove the inliers, extract/subtract the rest
		extract.setNegative(true);
		extract.filter(*cloud);
		//extract_normals.setNegative(true);
		//extract_normals.setInputCloud(cloud_normals);
		//extract_normals.setIndices(inliers_plane);
		//extract_normals.filter(*cloud_normals2);

		std::cout << " done." << std::endl;
	}


	void Primitive3::setVisualizeOnOff(bool visual)
	{
		visualizeOnOff_ = visual;
	}



	/* PointCloud utils */

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
		std::array<float, 2> arr = {1000.0, -1000.0};
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

	std::array<float, 6> Primitive3::getPointCloudRanges(const pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		std::array<float, 6> arr = { 1000.0, -1000.0, 1000.0, -1000.0, 1000.0, -1000.0};
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



	/// PRIMITIVE3D3

	void Primitive3d3::addSubPrimitive(Primitive3d1* p)
	{
		subprims_.push_back(p);
		are_subprims_custom_ = true;
	}

	/* Reshapes a primitive based on an heuristic. */
	void Primitive3d3::heuristic(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr, pcl::SACSegmentation<pcl::PointXYZ>* seg, HEURISTIC heu)
	{
		if (!cloud_->points.empty()) {
			cloud_->clear();
		}
		if (!subprims_.empty()) {
			for (auto sp : subprims_) {
				delete sp;
			}
			subprims_.clear();
		}

		for (auto cloud : cloud_arr) {
			if (!are_subprims_custom_) {
				this->cut(cloud, seg);
			}
		}		
		
		/*if (!this->heuristic_check(heu)) {
			std::cout << "The number of sub-primitives does not match the Heuristic." << std::endl;
			return;
		}*/

		this->heuristic_prim(heu);

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
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
	}

}