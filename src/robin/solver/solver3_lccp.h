#pragma once
#include "solver3.h"
#include "robin/sensor/sensor3.h"

#include <vector>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace robin
{
	class Solver3LCCP :
		public Solver3
	{
	public:
		Solver3LCCP() {}
		~Solver3LCCP() {}

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, std::string draw = "wireframe") const;

	protected:
		std::vector<int> pc_sizes_;
		std::vector<uint32_t> labels_;
		std::vector<std::array<float, 3>> centroids_;

		void segment();

		std::vector<int> getCentroids(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids);
		uint32_t selectCentroid(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids, const std::vector<int>& pc_sizes);
		void getLabeledCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label);
	};
}