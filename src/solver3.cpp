#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
	}

	Solver3::~Solver3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPointCloud() const {
		return cloud_;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;
	}

	void Solver3::setSegmentation(robin::Method3 seg_method)
	{
		seg_method_ = seg_method;
	}

	void Solver3::setUseNormals(bool seg_normals)
	{
		seg_normals_ = seg_normals;
	}

	void Solver3::setPlaneRemoval(bool seg_plane_removal)
	{
		seg_plane_removal_ = seg_plane_removal;
	}

	std::vector<robin::Sensor3*> Solver3::getSensors() const
	{
		return sensors_;
	}

	void Solver3::solve(robin::Primitive3& prim)
	{
		primitive_ = &prim;	

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		for (Sensor3* s : sensors_) {
			s->captureFrame();
			s->crop();
			s->downsample();
			*cloud_ += *s->getPointCloud();
		}

		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			std::cerr << "* Not enough points to perform segmentation." << std::endl;
			//std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			return;
		}

		this->segment();
	}

	void Solver3::segment()
	{
		switch (seg_method_) {
		case Method3::SEGMENTATION_SAC:
			this->segmentSAC();
			break;
		case Method3::SEGMENTATION_LCCP:
			this->segmentLCCP();
			this->segmentSAC();
			break;
		default:
			std::cerr << "Invalid Method3!" << std::endl;
			return;
		}
	}

	void Solver3::segmentSAC()
	{
		// Check whether a Primitive3 has been provided
		if (primitive_ != nullptr) {
			// Perform initial plane removal
			if (seg_plane_removal_) {
				robin::Primitive3Plane plane;
				if (seg_obj_ptr_ != nullptr) {					
					plane.fit(cloud_, seg_obj_ptr_);
				}
				else{
					plane.fit(cloud_, seg_normals_);
				}
			}

			// Check whether an instance of Segmentation has been provided.
			if (seg_obj_ptr_ != nullptr) {
				std::cout << "Segmentation object has been provided!" << std::endl;
				primitive_->fit(cloud_, seg_obj_ptr_);				
			}
			else {
				std::cout << "NO segmentation object has been provided." << std::endl;
				primitive_->fit(cloud_, seg_normals_);				
			}
		}
	}

	void Solver3::segmentLCCP()
	{	
		// Supervoxel Parameters
		float voxel_resolution = 0.001f; //0.0075f;
		float seed_resolution = 0.03f; //0.03f;
		float color_importance = 0.0f; //0.0f;
		float spatial_importance = 1.0f; //1.0f;
		float normal_importance = 4.0f; //4.0f;
		bool use_single_cam_transform = false;
		bool use_supervoxel_refinement = false;

		// LCCPSegmentation object Parameters
		float concavity_tolerance_threshold = 10;
		float smoothness_threshold = 0.1;
		std::uint32_t min_segment_size = 0;
		bool use_extended_convexity = false;
		bool use_sanity_criterion = false;

		float normals_scale(seed_resolution / 2.0);
		unsigned int k_factor = 0;
		if (use_extended_convexity)
			k_factor = 1;

		// Convert the cloud XYZ to XYZRGBA
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::copyPointCloud(*cloud_, *cloud_rgba);

		// Preparation of Input: Supervoxel Oversegmentation
		pcl::SupervoxelClustering<pcl::PointXYZRGBA> supervox(voxel_resolution, seed_resolution);
		supervox.setUseSingleCameraTransform(use_single_cam_transform);
		supervox.setInputCloud(cloud_rgba);
		supervox.setColorImportance(color_importance);
		supervox.setSpatialImportance(spatial_importance);
		supervox.setNormalImportance(normal_importance);
		std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;

		// Extract the supervoxels
		PCL_INFO("Extracting supervoxels\n");
		supervox.extract(supervoxel_clusters);

		if (use_supervoxel_refinement) {
			PCL_INFO("Refining supervoxels\n");
			supervox.refineSupervoxels(2, supervoxel_clusters);
		}
		std::stringstream temp;
		temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
		PCL_INFO(temp.str().c_str());

		PCL_INFO("Getting supervoxel adjacency\n");
		std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
		supervox.getSupervoxelAdjacency(supervoxel_adjacency);

		/// The Main Step: Perform LCCPSegmentation
		pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
		PCL_INFO("Starting Segmentation\n");
		lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
		lccp.setSanityCheck(use_sanity_criterion);
		lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
		lccp.setKFactor(k_factor);
		lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
		lccp.setMinSegmentSize(min_segment_size);
		lccp.segment();

		PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
		pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud(supervox.getLabeledCloud());
		pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud(sv_labeled_cloud->makeShared());
		lccp.relabelCloud(*lccp_labeled_cloud);

		std::vector<int> segments;
		std::vector<uint32_t> labels;
		std::vector<std::array<float, 3>> centroids;
		std::vector<int> label_count = getCentroidsLCCP(*lccp_labeled_cloud, labels, centroids);
		std::cout << "  Nr. Segments: " << labels.size() << std::endl;

		//this->visualizeCentroidsLCCP(centroids, labels, label_count, viewer);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);

		uint32_t lbl = this->selectCentroidLCCP(labels, centroids);
		this->getLabeledCloudLCCP(*lccp_labeled_cloud, *cloud_segmented, lbl);

		cloud_->clear();
		pcl::copyPointCloud(*cloud_segmented, *cloud_);
		std::cout << "PointCloud after LCCP: [" << lbl << "] " << cloud_segmented->points.size() << " data points (in xx ms)." << std::endl;
	}

	std::vector<int> Solver3::getCentroidsLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids)
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

	uint32_t Solver3::selectCentroidLCCP(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids)
	{
		float d(10.0);
		uint32_t lbl(std::pow(2, 32) - 1);
		for (size_t i(0); i < centroids.size(); ++i) {
			float d_temp = std::sqrt(std::pow(centroids[i][0], 2) + std::pow(centroids[i][1], 2));
			if (0.0001 < d_temp && d_temp < d) {
				lbl = labels[i];
				d = d_temp;
			}
		}
		return lbl;
	}

	void Solver3::getLabeledCloudLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label)
	{
		for (auto p : cloud_lccp.points) {
			if (p.label == label)
				cloud_seg.points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
		}
	}

	void Solver3::visualizeCentroidsLCCP(std::vector<std::array<float, 3>> centroids, std::vector<uint32_t> labels, std::vector<int> label_count, pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		float txt_gray_lvl(1.0);
		int vp(0);

		std::string id;
		for (int i(0); i < centroids.size(); ++i) {
			pcl::PointXYZ pFUL(centroids[i][0] - 0.005, centroids[i][1] - 0.005, centroids[i][2] - 0.005);
			pcl::PointXYZ pFUR(centroids[i][0] + 0.005, centroids[i][1] - 0.005, centroids[i][2] - 0.005);
			pcl::PointXYZ pFDL(centroids[i][0] - 0.005, centroids[i][1] + 0.005, centroids[i][2] - 0.005);
			pcl::PointXYZ pFDR(centroids[i][0] + 0.005, centroids[i][1] + 0.005, centroids[i][2] - 0.005);

			pcl::PointXYZ pBUL(centroids[i][0] - 0.005, centroids[i][1] - 0.005, centroids[i][2] + 0.005);
			pcl::PointXYZ pBUR(centroids[i][0] + 0.005, centroids[i][1] - 0.005, centroids[i][2] + 0.005);
			pcl::PointXYZ pBDL(centroids[i][0] - 0.005, centroids[i][1] + 0.005, centroids[i][2] + 0.005);
			pcl::PointXYZ pBDR(centroids[i][0] + 0.005, centroids[i][1] + 0.005, centroids[i][2] + 0.005);

			viewer->addLine<pcl::PointXYZ>(pFUL, pFUR, int(255) * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FU", vp);
			viewer->addLine<pcl::PointXYZ>(pFUR, pFDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FR", vp);
			viewer->addLine<pcl::PointXYZ>(pFDR, pFDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FD", vp);
			viewer->addLine<pcl::PointXYZ>(pFDL, pFUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "FL", vp);

			viewer->addLine<pcl::PointXYZ>(pBUL, pBUR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BU", vp);
			viewer->addLine<pcl::PointXYZ>(pBUR, pBDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BR", vp);
			viewer->addLine<pcl::PointXYZ>(pBDR, pBDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BD", vp);
			viewer->addLine<pcl::PointXYZ>(pBDL, pBUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "BL", vp);

			viewer->addLine<pcl::PointXYZ>(pBUL, pFUL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "LU", vp);
			viewer->addLine<pcl::PointXYZ>(pBDL, pFDL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "LD", vp);
			viewer->addLine<pcl::PointXYZ>(pBUR, pFUR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "RU", vp);
			viewer->addLine<pcl::PointXYZ>(pBDR, pFDR, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, std::to_string(i) + "RD", vp);

			std::cout << "    l: " << labels[i] <<
				" pts: " << label_count[i] <<
				" x: " << centroids[i][0] <<
				" y: " << centroids[i][1] <<
				" z: " << centroids[i][2] << std::endl;
		}
	}
}