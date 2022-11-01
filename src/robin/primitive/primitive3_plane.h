#pragma once
#include "primitive3.h"

#include <string>
#include <cstdlib>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/pca.h>

namespace robin
{
	enum class PLANE_TYPE {
		DEFAULT,
		PERPENDICULAR,
		PARALLEL
	};
	
	class Primitive3Plane :
		public Primitive3d2
	{
	public:

		Primitive3Plane();

		/* Primitive3Plane can be initialized by "DEFAULT" or by "PERPENDICULAR" or "PARALLEL" types. */
		Primitive3Plane(PLANE_TYPE type, Eigen::Vector3f v, float angle);

		~Primitive3Plane();

		void reset();

		void setCoefficients(std::vector<float> v);

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;		

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	protected:
		PLANE_TYPE type_;

		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients if necessary. */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();
	};
}