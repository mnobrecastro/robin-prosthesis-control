#pragma once
#include "robin/sensor/sensor.h"
#include "robin/primitive/primitive3.h"
#include "robin/primitive/primitive3_sphere.h"
#include "robin/primitive/primitive3_cylinder.h"
#include "robin/primitive/primitive3_cuboid.h"
#include "robin/sensor/tactor.h"

#include <vector>
#include <cmath>
#include <thread>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace robin
{
	enum class FEEDBACK_CLOUD {
		DIST_TO_HULL,
		DIST_TO_COM
	};

	enum class FEEDBACK_PRIM {
		TYPE
	};

	enum class PRIM_TYPE {
		SPHERE,
		CYLINDER,
		CUBOID
	};

	namespace feedback
	{
		class Feedback
		{
		public:
			Feedback();
			~Feedback();

			void addTactor(Tactor* s);

			pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

			/* Rendering of the feedback on a PCLVisualizer (ex. Hull). */
			virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

			virtual void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, robin::FEEDBACK_CLOUD fc);

			virtual void addPrimitive3(const robin::Primitive3d3* prim, robin::FEEDBACK_PRIM fp);

			virtual void setActive(bool b) { is_active_ = b; }

			virtual void run();
			
		protected:
			Tactor* tactor_;

			bool is_active_ = true;

			/* Point cloud and point on ConvHull for rendering purposes */
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_;
			pcl::PointXYZ pHull_;
			bool in_hull_ = false;
			robin::PRIM_TYPE prim_type_;

			pcl::PointCloud<pcl::PointXYZ>::Ptr quick_hull_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr find_hull_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ p0, pcl::PointXYZ p1) const;

			bool is_inside_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr hull, const pcl::PointXYZ p, pcl::PointXYZ& pHull) const;
			int signum(float val) const;
			float distance_to_lineseg(const pcl::PointXYZ a, const pcl::PointXYZ b, const pcl::PointXYZ p, pcl::PointXYZ& pHull) const;
		};
	}
}