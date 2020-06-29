#pragma once
#include "primitive3.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

namespace robin
{
	class Primitive3Plane :
		public Primitive3d2
	{
	public:
		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

	protected:
		
	};
}