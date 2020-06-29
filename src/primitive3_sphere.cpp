#include "primitive3_sphere.h"

namespace robin
{
	void Primitive3Sphere::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		viewer->addSphere(*coefficients_, "cylinder");
	}


	void Primitive3Sphere::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		if (normals) {
			fit_sample_consensus_with_normals(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_SPHERE);
		}
		else {
			fit_sample_consensus(cloud, pcl::SAC_RANSAC, pcl::SACMODEL_SPHERE);
		}

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties ****/
	}

	void Primitive3Sphere::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		seg->setModelType(pcl::SACMODEL_SPHERE);
		if (typeid(seg) == typeid(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>)) {
			fit_sample_consensus_with_normals(cloud, seg);
		}
		else {
			fit_sample_consensus(cloud, seg);
		}

		/**** 1. Update/correct model coeffficients ****/
		/**** 2. Update object properties ****/
	}
}