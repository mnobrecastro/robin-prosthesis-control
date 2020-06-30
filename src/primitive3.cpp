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


	pcl::PointCloud<pcl::PointXYZ>::Ptr Primitive3::getPointCloud() const {
		return cloud_;
	}


	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		//time.tic();
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SAC_MODEL);
		seg.setMethodType(SAC_METHOD);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.0025);
		//if (SAC_MODEL == pcl::SACMODEL_CYLINDER || SAC_MODEL == pcl::SACMODEL_SPHERE) {
		seg.setRadiusLimits(0.005, 0.050);
		//}

		this-> fit_sample_consensus(cloud, seg);

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties (incl. cloud boundaries? "bounds_plane") ****/
	}

	void Primitive3::fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ> seg)
	{
		//time.tic();
		// Copy the segmentation object
		seg.setInputCloud(cloud);
		//if (seg.getModelType() == pcl::SACMODEL_CYLINDER || seg.getModelType() == pcl::SACMODEL_SPHERE) {
		//seg.setRadiusLimits(0.005, 0.050); /* Can be read from the hand object*/
		//}

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);
		//std::cerr << "Plane coefficients: " << *coefficients_ << std::endl;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_);
		//std::cerr << "PointCloud representing the planar component: " << cloud_->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;

		// Remove the inliers, extract/subtract the rest
		extract.setNegative(true);
		extract.filter(*cloud);

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties (incl. cloud boundaries? "bounds_plane") ****/
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL)
	{
		//time.tic();
		// Create the segmentation object
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SAC_MODEL);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(SAC_METHOD);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.0025);
		if (SAC_MODEL == pcl::SACMODEL_CYLINDER || SAC_MODEL == pcl::SACMODEL_SPHERE) {
			seg.setRadiusLimits(0.005, 0.050);
		}

		this->fit_sample_consensus_with_normals(cloud, seg);
	}

	void Primitive3::fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg)
	{
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		// Estimate point normals
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		std::cout << "Computing normals...";
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);
		std::cout << " done." << std::endl;

		//time.tic();
		// Copy the segmentation object
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals);
		//if (seg.getModelType() == pcl::SACMODEL_CYLINDER || seg.getModelType() == pcl::SACMODEL_SPHERE) {
		//seg.setRadiusLimits(0.005, 0.050);
		//}

		// Obtain the plane inliers and coefficients
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.segment(*inliers, *coefficients_);
		//std::cerr << "Plane coefficients: " << *coefficients_ << std::endl;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_);
		//std::cerr << "PointCloud representing the planar component: " << cloud_->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;


		// Remove the inliers, extract/subtract the rest
		extract.setNegative(true);
		extract.filter(*cloud);
		//extract_normals.setNegative(true);
		//extract_normals.setInputCloud(cloud_normals);
		//extract_normals.setIndices(inliers_plane);
		//extract_normals.filter(*cloud_normals2);
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
}