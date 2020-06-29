#pragma once
#include "primitive.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace robin
{
	class Primitive3 :
		public Primitive
	{
	public:
		Primitive3();
		~Primitive3();

		pcl::ModelCoefficients::Ptr getCoefficients();

		struct Properties getProperties();

		void setCoefficients();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;


		virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const {}


		/* Receives a PointCloud object by reference and extracts/segments it by fitting to it. */
		virtual void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool seg_normals) {}

		/* Receives a PointCloud and a segmentation object by reference and extracts/segments it by fitting to it. */
		virtual void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ> seg) {}

	protected:
		struct Properties {} properties_;
		pcl::ModelCoefficients::Ptr coefficients_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
				
		void fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL);
		void fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ> seg);

		void fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL);
		void fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg);
	};

	class Primitive3d1 :
		public Primitive3
	{
		/* Dummy Class */
	};

	class Primitive3d2 :
		public Primitive3
	{
		/* Dummy Class */
	};

	class Primitive3d3 :
		public Primitive3
	{
		/* Dummy Class */
	};
}