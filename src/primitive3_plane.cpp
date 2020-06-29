#include "primitive3_plane.h"

namespace robin
{
	void Primitive3Plane::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addPlane(*coefficients_, "plane");
	}

	void Primitive3Plane::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_NORMAL_PLANE);
		}

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties ****/
	}
}