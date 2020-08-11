#pragma once
#include "solver.h"
#include "sensor3.h"
#include "primitive3_plane.h"

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
	enum class Method3 {
		SEGMENTATION_SAC,
		SEGMENTATION_LCCP
	};

	class Solver3 :
		public Solver
	{
	public:
		Solver3();
		//Solver3(robin::Primitive3*);
		//Solver3(robin::Method3);
		~Solver3();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

		void addSensor(robin::Sensor3*);

		void setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj);

		void setSegmentation(robin::Method3 seg_method);

		void setUseNormals(bool seg_normals);

		void setPlaneRemoval(bool seg_plane_removal);

		void setCrop(float, float, float, float, float, float);

		void setDownsample(float);

		std::vector<robin::Sensor3*> getSensors() const;

		void solve(robin::Primitive3& prim);

		void Solver3::visualizeLCCP(pcl::visualization::PCLVisualizer::Ptr viewer, std::string draw="wireframe") const;

	protected:
		std::vector<robin::Sensor3*> sensors_;
		Primitive3* primitive_ = nullptr;

		/* Point cloud (temp) that can be manipulated */
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

		bool filterOnOff_ = false;
		std::array<float, 6> limits_;
		bool downsampleOnOff_ = false;
		float voxel_size_ = 0.005f;

		pcl::PointCloud<pcl::PointXYZ>::Ptr trimPointCloud();
		void crop();
		void downsample();

		unsigned int MIN_POINTS_PROCEED_ = 100;

		robin::Method3 seg_method_;
		bool seg_normals_ = false;
		bool seg_plane_removal_ = false;
		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_ptr_ = nullptr;

		void segment();
		void segmentSAC();
		void segmentLCCP();

		std::vector<int> lccp_pc_sizes_;
		std::vector<uint32_t> lccp_labels_;
		std::vector<std::array<float, 3>> lccp_centroids_;

		std::vector<int> getCentroidsLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud, std::vector<uint32_t>& labels, std::vector<std::array<float, 3>>& centroids);
		uint32_t selectCentroidLCCP(const std::vector<uint32_t>& labels, const std::vector<std::array<float, 3>>& centroids, const std::vector<int>& pc_sizes);
		void getLabeledCloudLCCP(const pcl::PointCloud<pcl::PointXYZL>& cloud_lccp, pcl::PointCloud<pcl::PointXYZ>& cloud_seg, uint32_t label);
	};
}