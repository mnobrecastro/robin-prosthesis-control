#pragma once
#include "robin/sensor/sensor.h"
#include "robin/primitive/primitive.h"

#include <vector>

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
		CLOSEST_POINT,
		CENTER_OF_MASS
	};

	enum class FEEDBACK_PRIM {
		SOMETHING
	};

	namespace feedback
	{
		class Feedback
		{
		public:
			Feedback();
			~Feedback() {}

			void addSensor(Sensor* s);

			pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

			/* Rendering of the feedback on a PCLVisualizer (ex. Hull). */
			virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

			virtual void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, robin::FEEDBACK_CLOUD fc);

			virtual void fromPrimitive(const robin::Primitive* prim, robin::FEEDBACK_PRIM fp);
			
		protected:
			std::vector<Sensor*> sensors_;

			/* Point cloud and vector for rendering purposes */
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
			pcl::PointXYZ arrow_;

			pcl::PointCloud<pcl::PointXYZ>::Ptr quick_hull_2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr find_hull_2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ p0, pcl::PointXYZ p1);

			bool is_inside_hull(pcl::PointXYZ p, pcl::PointXYZ& d);

		};
	}
}