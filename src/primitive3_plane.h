#pragma once
#include "primitive3.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/pca.h>

namespace robin
{
	class Primitive3Plane :
		public Primitive3d2
	{
	public:
		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	protected:
		struct Properties {
			float center_x = 0.0;
			float center_y = 0.0;
			float center_z = 0.0;
			float normal_x = 0.0;
			float normal_y = 0.0;
			float normal_z = 0.0;
			float e0_x = 0.0;
			float e0_y = 0.0;
			float e0_z = 0.0;
			float e1_x = 0.0;
			float e1_y = 0.0;
			float e1_z = 0.0;
			float width = 0.0;
			float height = 0.0;
		} properties_;

		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients if necessary. */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();
	};
}