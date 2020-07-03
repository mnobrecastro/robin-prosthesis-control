#pragma once
#include "primitive3.h"
#include "primitive3_plane.h"

#include <vector>
#include <typeinfo>

namespace robin
{
	class Primitive3Cuboid :
		public Primitive3d3
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
			float width = 0.0;
			float height = 0.0;
			float depth = 0.0;
			float axis_x = 0.0;
			float axis_y = 0.0;
			float axis_z = 0.0;
		} properties_;

		std::vector<Primitive3Plane*> planes_;

		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients if necessary. */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();		
	};
}