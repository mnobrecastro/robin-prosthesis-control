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

		void addSensor(robin::Sensor3*);

		void setPrimitive(robin::Primitive3&);


		void setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj);

		void setSegmentation(robin::Method3 seg_method);

		void setUseNormals(bool seg_normals);

		void setPlaneRemoval(bool seg_plane_removal);


		void activate();

	protected:
		std::vector<robin::Sensor3*> sensors_;
		Primitive3* primitive_ = nullptr;

		/* Point cloud (temp) that can be manipulated */
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

		unsigned int MIN_POINTS_PROCEED_ = 100;

		pcl::visualization::PCLVisualizer::Ptr viewer_;
		pcl::visualization::PCLVisualizer::Ptr initVisualizer();

		void startSensors(bool);

		robin::Method3 seg_method_;
		bool seg_normals_ = false;
		bool seg_plane_removal_ = false;
		pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_ptr_ = nullptr;


		void segment();
		void segmentSAC();
		void segmentLCCP();
	};
}