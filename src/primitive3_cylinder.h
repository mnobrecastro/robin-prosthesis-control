#pragma once
#include "primitive3.h"

namespace robin
{
	class Primitive3Cylinder :
		public Primitive3d3
	{
	public:
		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

	private:

	};
}