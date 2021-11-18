#pragma once
#include "primitive3.h"
#include "primitive3_plane.h"
#include "primitive3_line.h"

#include <vector>
#include <array>
#include <typeinfo>
#include <cmath>

#define MAX_SUBPRIMS 1

namespace robin
{
	class Primitive3Cuboid :
		public Primitive3d3
	{
	public:
		Primitive3Cuboid();
		~Primitive3Cuboid();

		void reset();

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

		/* Render the intersection point p obtained by the cam axis. */
		void setIntersectionPoint(Eigen::Vector3f p) { inters_point_ = p; }

		/* Set the index for a specific face to be highlighted in the viewer. */
		void setFaceHighlight(int i) { view_face_idx_ = i; }

		void setCoefficients(std::vector<float> v);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals);

		void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

	protected:
		/*struct Properties {
			float center_x = 0.0;
			float center_y = 0.0;
			float center_z = 0.0;
			float width = 0.0;
			float height = 0.0;
			float depth = 0.0;
			float axis_x = 0.0;
			float axis_y = 0.0;
			float axis_z = 0.0;
		} properties_;*/

		std::vector<Primitive3Plane*> planes_;

		std::array<float,12> plot_;

		Eigen::Vector3f inters_point_;		
		int view_face_idx_ = -1; // Face to be highlighted in the viewer (-1: None)

		/* Automatic Cuboid face highlighting based on ray casting. */
		size_t face_highlight();

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

		/* Checks if the heuristic is valid. */
		bool is_heuristic_valid();
	};
}