#pragma once
#include "primitive3.h"

#include <typeinfo>

namespace robin
{
	class Primitive3Cylinder :
		public Primitive3d3
	{
	public:
		Primitive3Cylinder();
		~Primitive3Cylinder();

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void reset();

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	protected:
		/*struct Properties {
			float radius = 0.0;
			float center_x = 0.0;
			float center_y = 0.0;
			float center_z = 0.0;
			float axis_x = 0.0;
			float axis_y = 0.0;
			float axis_z = 0.0;
		} properties_;*/

		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients if necessary. */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();
	};
}