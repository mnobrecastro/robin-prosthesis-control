#pragma once
#include "primitive3.h"

#include <typeinfo>

namespace robin
{
	class Primitive3Sphere :
		public Primitive3d3
	{
	public:
		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);
		
		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	private:

	};
}