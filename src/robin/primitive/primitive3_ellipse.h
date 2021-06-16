#pragma once
#include "primitive3.h"

namespace robin
{
	class Primitive3Ellipse :
		public Primitive3d1
	{
	public:
		Primitive3Ellipse();
		~Primitive3Ellipse();

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		void reset();

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);
		
	protected:
		/* Checks if the fit is valid. */
		bool is_fit_valid();

		/* Correct the obtained coefficients by converting a Ellipse3D model into a Cylinder (rendering suitable). */
		void correct_coefficients();

		/* Update the properties of the Primitive3. */
		void update_properties();
	};
}