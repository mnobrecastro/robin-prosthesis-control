#pragma once
#include "primitive3.h"
#include "primitive3_plane.h"

#include <vector>

namespace robin
{
	class Primitive3Cube :
		public Primitive3d3
	{
	public:
		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

	private:
		std::vector<Primitive3Plane*> planes_;
	};
}