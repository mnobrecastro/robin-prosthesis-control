#include "solver3_affnet.h"

namespace robin
{
	void Solver3AffNet::addSensor(std::shared_ptr<Webcam> cam)
	{
		sensor_ = cam;
		return;
	}
	
	void Solver3AffNet::segment()
	{
		//pc_sizes_.clear();
		//labels_.clear();
		//centroids_.clear();

		//// check whether the pointcloud has enough points to proceed
		//if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
		//	primitive_->reset();
		//	std::cerr << "* not enough points to perform sac segmentation.\n" << std::endl;
		//	return;
		//}

		// RS Camera Intrinsics
		const float rx(424), ry(240); // Resolution
		const float cx(211.937), cy(122.685); // Center dist
		const float f(211.357); // Focal lenght
		const float scaling(1000.0); // Scaling factor

		cv::Mat affordance_ch;
		cv::extractChannel(im_mask_, affordance_ch, 0); // Graspable-only
		cv::Mat affordance_mask;
		cv::threshold(affordance_ch, affordance_mask, 1, 255, cv::THRESH_BINARY);

		cv::Mat im_blobs, stats, centroids; // Graspable-only
		int n_blobs = cv::connectedComponentsWithStats(affordance_mask, im_blobs, stats, centroids, 8);
		double min_dist(std::numeric_limits<float>::max());
		int blob_idx(0); // find the index of the center-most blob
		int min_area(0.01 * rx * ry); // 10% of the image area in px
		for (int k(1); k < n_blobs; ++k) {
			double dist = std::sqrt((centroids.at<double>(k, 0) - cx) * (centroids.at<double>(k, 0) - cx) + (centroids.at<double>(k, 1) - cy) * (centroids.at<double>(k, 1) - cy));
			if (dist < min_dist && stats.at<int>(k, cv::CC_STAT_AREA) > min_area) {
				blob_idx = k;
				min_dist = dist;
			}
		}

		// Choose the center-most blob among those found by the connectedComponents
		cv::Mat affordance_blob = cv::Mat::zeros(im_blobs.size().height, im_blobs.size().width, CV_8UC1);
		if (blob_idx > 0) {
			for (size_t y(0); y < im_blobs.size().height; ++y) {
				for (size_t x(0); x < im_blobs.size().width; ++x) {
					if (im_blobs.at<int>(y, x) == blob_idx) {
						affordance_blob.at<uchar>(y, x) = 255;
					}
				}
			}
		}

		// Reconstruct the point cloud by masking the depthdata with the affordance blob
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask(new pcl::PointCloud<pcl::PointXYZ>());
		cv::Mat affordance_depth;
		im_depth_.copyTo(affordance_depth, affordance_blob);
		for (size_t x(0); x < affordance_depth.size().width; ++x) {
			for (size_t y(0); y < affordance_depth.size().height; ++y) {
				// Recall BGR ordering				
				float pZ = (affordance_depth.at<cv::Vec3b>(y, x).val[0] + affordance_depth.at<cv::Vec3b>(y, x).val[1] + affordance_depth.at<cv::Vec3b>(y, x).val[2]) / scaling;
				float pX = (x - cx) * pZ / f;
				float pY = (y - cy) * pZ / f;
				if (pZ > 0.100) {
					cloud_mask->push_back(pcl::PointXYZ(pX, pY, pZ));
				}
			}
		}
		std::cout << "mask: " << cloud_mask->size() << "\n";

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_mask;
		sor_mask.setInputCloud(cloud_mask);
		sor_mask.setMeanK(20);
		sor_mask.setStddevMulThresh(1.0); //1.0
		sor_mask.filter(*cloud_mask);
		
		cloud_ = cloud_mask;
		
		//pc_sizes_.clear();
		//labels_.clear();
		//centroids_.clear();

		//// Check whether the PointCloud has enough points to proceed
		//if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
		//	primitive_->reset();
		//	std::cerr << "* Not enough points to perform SAC segmentation." << std::endl;
		//	return;
		//}

		//// Estimate point normals
		//bool use_normal_estimation(false);
		//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
		//if (use_normal_estimation) {
		//	std::cout << "Computing normals...";
		//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		//	ne.setSearchMethod(kdtree);
		//	ne.setInputCloud(cloud_);
		//	ne.setKSearch(50);
		//	ne.compute(*cloud_normals);
		//	std::cout << " done." << std::endl;
		//}

		//// Convert the cloud XYZ to XYZRGBA
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//pcl::copyPointCloud(*cloud_, *cloud_rgba);

		//// Supervoxel Oversegmentation
		//float voxel_resolution = voxel_size_;//0.0025f; //0.0075f;
		//float seed_resolution = 4 * voxel_size_;//0.010f; //0.03f;
		//pcl::SupervoxelClustering<pcl::PointXYZRGBA> supervox(voxel_resolution, seed_resolution);
		//supervox.setInputCloud(cloud_rgba);
		//supervox.setUseSingleCameraTransform(false);
		//supervox.setColorImportance(0.0f); //0.0f
		//supervox.setSpatialImportance(1.0f); //1.0f
		//supervox.setNormalImportance(4.0f); //4.0f
		//if (use_normal_estimation) { supervox.setNormalCloud(cloud_normals); }

		//// Extract the supervoxels
		//bool use_supervoxel_refinement(false);
		//std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;
		//supervox.extract(supervoxel_clusters);
		//if (use_supervoxel_refinement) {
		//	// Refining supervoxels
		//	supervox.refineSupervoxels(2, supervoxel_clusters);
		//}
		//// Get supervoxel adjacency
		//std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
		//supervox.getSupervoxelAdjacency(supervoxel_adjacency);

		//// Perform the LCCPSegmentation
		//pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
		//lccp.setConcavityToleranceThreshold(10.0);
		//lccp.setSanityCheck(false);
		//lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, 0.1); // smoothness_threshold
		//lccp.setKFactor((unsigned int)1); //0 // k_factor = 1 for extended_convexity
		//lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
		//lccp.setMinSegmentSize((unsigned int)0);
		//lccp.segment();

		//// Interpolation voxel cloud (input cloud and relabeling)
		//pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud(supervox.getLabeledCloud());
		//pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud(sv_labeled_cloud->makeShared());
		//lccp.relabelCloud(*lccp_labeled_cloud);

		//// Retrieve the number of labels and respective clouds
		//pc_sizes_ = this->getCentroids(*lccp_labeled_cloud, labels_, centroids_);
		//uint32_t label = this->selectCentroid(labels_, centroids_, pc_sizes_);

		//if (label != std::pow(2, 32) - 1) {
		//	// Retrieve the PointCloud with the smallest centroid projection about the global Z-axis
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
		//	this->getLabeledCloud(*lccp_labeled_cloud, *cloud_segmented, label);
		//	cloud_->clear();
		//	pcl::copyPointCloud(*cloud_segmented, *cloud_);

		//	std::cout << "PointCloud after LCCP: [" << supervoxel_clusters.size() << " sv -> " << label << "/" << labels_.size() << "] " << cloud_segmented->points.size() << " data points (in xx ms)." << std::endl;
		//}
		//else {
		//	// Fails if no centroid was selected as the biggest uint32_t (2**32-1) is retrieved
		//	std::cout << "LCCP failed to find a suitable point cloud among all labeled ones." << std::endl;
		//	return;
		//}

		///
		this->resample();

		Solver3::segment();
	}

	std::vector<int> Solver3AffNet::getCentroids(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids)
	{
		std::vector<int> counts;
		for (auto p : cloud.points) {
			bool is_in(false);
			size_t i(0);
			while (!is_in && i < labels.size()) {
				if (labels[i] == p.label) {
					is_in = true;
				}
				else {
					++i;
				}
			}
			if (!is_in) {
				labels.push_back(p.label);
				counts.push_back(0);
				centroids.push_back({ 0.0, 0.0, 0.0 });
			}
			counts[i] += 1;
			centroids[i][0] += p.x;
			centroids[i][1] += p.y;
			centroids[i][2] += p.z;
		}
		for (size_t i(0); i < labels.size(); ++i) {
			centroids[i][0] /= counts[i];
			centroids[i][1] /= counts[i];
			centroids[i][2] /= counts[i];
		}
		return counts;
	}

	uint32_t Solver3AffNet::selectCentroid(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids, const std::vector<int>& pc_sizes)
	{
		float d(10.0);
		uint32_t label(std::pow(2, 32) - 1); // Needs a guard to verify if this maximum value is the output
		for (size_t i(0); i < centroids.size(); ++i) {
			if (pc_sizes[i] > MIN_POINTS_PROCEED_) {
				float d_temp = std::sqrt(std::pow(centroids[i][0], 2) + std::pow(centroids[i][1], 2));
				if (0.0001 < d_temp && d_temp < d) {
					label = labels[i];
					d = d_temp;
				}
			}
		}
		return label;
	}

	void Solver3AffNet::getLabeledCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label)
	{
		for (auto p : cloud_lccp.points) {
			if (p.label == label)
				cloud_seg.points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
		}
	}

	void Solver3AffNet::visualize(pcl::visualization::PCLVisualizer::Ptr viewer, std::string draw) const
	{
		int vp(0);

		std::string id;
		for (int i(0); i < centroids_.size(); ++i) {

			if (pc_sizes_[i] > MIN_POINTS_PROCEED_) {
				if (draw == "wireframe") {
					pcl::PointXYZ pFUL(centroids_[i][0] - 0.005, centroids_[i][1] - 0.005, centroids_[i][2] - 0.005);
					pcl::PointXYZ pFUR(centroids_[i][0] + 0.005, centroids_[i][1] - 0.005, centroids_[i][2] - 0.005);
					pcl::PointXYZ pFDL(centroids_[i][0] - 0.005, centroids_[i][1] + 0.005, centroids_[i][2] - 0.005);
					pcl::PointXYZ pFDR(centroids_[i][0] + 0.005, centroids_[i][1] + 0.005, centroids_[i][2] - 0.005);

					pcl::PointXYZ pBUL(centroids_[i][0] - 0.005, centroids_[i][1] - 0.005, centroids_[i][2] + 0.005);
					pcl::PointXYZ pBUR(centroids_[i][0] + 0.005, centroids_[i][1] - 0.005, centroids_[i][2] + 0.005);
					pcl::PointXYZ pBDL(centroids_[i][0] - 0.005, centroids_[i][1] + 0.005, centroids_[i][2] + 0.005);
					pcl::PointXYZ pBDR(centroids_[i][0] + 0.005, centroids_[i][1] + 0.005, centroids_[i][2] + 0.005);

					viewer->addLine<pcl::PointXYZ>(pFUL, pFUR, 255, 255, 255, std::to_string(i) + "FU", vp);
					viewer->addLine<pcl::PointXYZ>(pFUR, pFDR, 255, 255, 255, std::to_string(i) + "FR", vp);
					viewer->addLine<pcl::PointXYZ>(pFDR, pFDL, 255, 255, 255, std::to_string(i) + "FD", vp);
					viewer->addLine<pcl::PointXYZ>(pFDL, pFUL, 255, 255, 255, std::to_string(i) + "FL", vp);

					viewer->addLine<pcl::PointXYZ>(pBUL, pBUR, 255, 255, 255, std::to_string(i) + "BU", vp);
					viewer->addLine<pcl::PointXYZ>(pBUR, pBDR, 255, 255, 255, std::to_string(i) + "BR", vp);
					viewer->addLine<pcl::PointXYZ>(pBDR, pBDL, 255, 255, 255, std::to_string(i) + "BD", vp);
					viewer->addLine<pcl::PointXYZ>(pBDL, pBUL, 255, 255, 255, std::to_string(i) + "BL", vp);

					viewer->addLine<pcl::PointXYZ>(pBUL, pFUL, 255, 255, 255, std::to_string(i) + "LU", vp);
					viewer->addLine<pcl::PointXYZ>(pBDL, pFDL, 255, 255, 255, std::to_string(i) + "LD", vp);
					viewer->addLine<pcl::PointXYZ>(pBUR, pFUR, 255, 255, 255, std::to_string(i) + "RU", vp);
					viewer->addLine<pcl::PointXYZ>(pBDR, pFDR, 255, 255, 255, std::to_string(i) + "RD", vp);
				}
				else if (draw == "marker") {
					pcl::ModelCoefficients marker_coef;
					marker_coef.values.push_back(centroids_[i][0]);
					marker_coef.values.push_back(centroids_[i][1]);
					marker_coef.values.push_back(centroids_[i][2]);
					marker_coef.values.push_back(0.005);
					viewer->addSphere(marker_coef, std::to_string(labels_[i]));
				}
			}
		}
	}
}