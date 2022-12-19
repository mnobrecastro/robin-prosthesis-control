#pragma once
#include "solver3.h"
#include "robin/sensor/sensor3.h"
#include "robin/sensor/webcam.h"

#include <vector>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace robin
{
	class Solver3AffNet :
		public Solver3
	{
	public:
		Solver3AffNet() {}
		~Solver3AffNet() {}

		void addSensor(std::shared_ptr<Webcam> cam);

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, std::string draw = "marker") const;

	protected:
		std::vector<int> pc_sizes_;
		std::vector<uint32_t> labels_;
		std::vector<std::array<float, 3>> centroids_;
		
		std::shared_ptr<Webcam> sensor_ = nullptr;

		cv::Mat im_src_;
		cv::Mat im_depth_;
		cv::Mat im_mask_;

		void segment();

		/* Remove the method from the interface. */
		using Solver3::addSensor;

		std::vector<int> getCentroids(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids);
		uint32_t selectCentroid(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids, const std::vector<int>& pc_sizes);
		void getLabeledCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label);
	};
}