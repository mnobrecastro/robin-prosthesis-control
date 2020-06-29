#include "primitive3_cylinder.h"

namespace robin
{
	void Primitive3Cylinder::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CYLINDER);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_CYLINDER);
		}

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties ****/
	}
}