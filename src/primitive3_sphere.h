#pragma once
#include "primitive3.h"
#include "primitive3_circle.h"

#include <typeinfo>

#define MAX_SUBPRIMS 1

namespace robin
{
	class Primitive3Sphere :
		public Primitive3d3
	{
	public:
		Primitive3Sphere();
		~Primitive3Sphere();

		void reset();

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);
		
		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	protected:
		struct Properties {
			float radius = 0.0;
			float center_x = 0.0;
			float center_y = 0.0;
			float center_z = 0.0;
		} properties_;

		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients if necessary. */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();

		/* Receives a PointCloud cut by reference and fits a sub-primitive to it. */
		void cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
		void cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

		void heuristic_laser_array_single();
		void heuristic_laser_array_cross();
		void heuristic_laser_array_star();
	};
}