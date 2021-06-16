#define MULTITHREADING

#pragma once
#include "solver.h"
#include "robin/sensor/sensor3.h"
#include "robin/primitive/primitive3_plane.h"
#include "robin/primitive/primitive3_sphere.h"
#include "robin/primitive/primitive3_cuboid.h"
#include "robin/primitive/primitive3_cylinder.h"

#include <vector>
#include <typeinfo>
#include <thread>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace robin
{
	class Solver3 :
		public Solver
	{
	public:
		Solver3();
		~Solver3();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRawColored() const;

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPreprocessed() const;

		void addSensor(robin::Sensor3*);

		void setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj);

		void setUseNormals(bool seg_normals);

		void setPlaneRemoval(bool seg_plane_removal);

		void setCrop(float, float, float, float, float, float);

		void setDownsample(float);

		void setResample(size_t order, float radius);

		void setFairSelection(bool fairness);

		std::vector<robin::Sensor3*> getSensors() const;

		void solve(robin::Primitive3d3*& prim);

		virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

	protected:
		std::vector<robin::Sensor3*> sensors_;
		Primitive3d3* primitive_ = nullptr;

		/* Point cloud (temp) that can be manipulated. */
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

		/* Raw coloured point cloud. */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_clr_;

		/* Pre-processed point cloud (after cropping and downsampling). */
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc_;

		bool filterOnOff_ = false;
		std::array<float, 6> limits_;
		void crop();

		bool downsampleOnOff_ = false;
		float voxel_size_ = 0.0025f;
		void downsample();

		bool resampleOnOff_ = false;
		size_t resamp_order_ = 0;
		float resamp_radius_ = 0.001f;
		void resample();

		/* To be used with multithreading Primitive3d3 fitting. */
		bool fairselectionOnOff_ = false;

		pcl::PointCloud<pcl::PointXYZ>::Ptr trimPointCloud();
		
		

		unsigned int MIN_POINTS_PROCEED_ = 100;

		bool seg_normals_ = false;
		bool seg_plane_removal_ = false;
		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_ptr_ = nullptr;

		virtual void segment();

		/* Primitive3's for multiple simultaneous fitting of a generic Primitive3d3. */
		robin::Primitive3d3* p_sph_ = new robin::Primitive3Sphere;
		robin::Primitive3d3* p_cub_ = new robin::Primitive3Cuboid;
		robin::Primitive3d3* p_cyl_ = new robin::Primitive3Cylinder;

		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_sph_;
		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_cub_;
		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_cyl_;
		
		/* Auxiliary Primitive fitting function for multithreading. */
		void fitPrimitive(robin::Primitive3d3*& prim, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj);
	};
}