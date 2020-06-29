#include "primitive3_cube.h"

namespace robin
{
	void Primitive3Cube::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		size_t n_planes(0), n_pts(cloud_->points.size());
		while (cloud_->points.size() > 0.01 * n_pts && n_planes < 3) {
			Primitive3Plane* plane_ptr(new Primitive3Plane());
			plane_ptr->fit(cloud, normals);
			planes_.push_back(plane_ptr);
			++n_planes;
		}

		/**** 0. Find cube dimensions ****/
		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties ****/
	}
}