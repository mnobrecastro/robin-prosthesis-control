#pragma once
#include "primitive.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#define LASER_ARRAY_SINGLE_SIZE 1
#define LASER_ARRAY_CROSS_SIZE 2
#define LASER_ARRAY_STAR_SIZE 4

namespace robin
{
	class Primitive3 :
		public Primitive
	{
	public:
		Primitive3();
		~Primitive3();

		Primitive3(const Primitive3& prim);

		bool isEmpty() { return isempty_; }

		float getProperty_center_x() { return properties_.center_x; }
		float getProperty_center_y() { return properties_.center_y; }
		float getProperty_center_z() { return properties_.center_z; }
		float getProperty_width() { return properties_.width; }
		float getProperty_height() { return properties_.height; }
		float getProperty_depth() { return properties_.depth; }
		float getProperty_axis_x() { return properties_.axis_x; }
		float getProperty_axis_y() { return properties_.axis_y; }
		float getProperty_axis_z() { return properties_.axis_z; }
		float getProperty_d() { return properties_.d; }
		float getProperty_radius() { return properties_.radius; }
		float getProperty_e0_x() { return properties_.e0_x; }
		float getProperty_e0_y() { return properties_.e0_y; }
		float getProperty_e0_z() { return properties_.e0_z; }
		float getProperty_e1_x() { return properties_.e1_x; }
		float getProperty_e1_y() { return properties_.e1_y; }
		float getProperty_e1_z() { return properties_.e1_z; }

		virtual void setCoefficients(std::vector<float> v) {}

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;
		pcl::ModelCoefficients::Ptr getCoefficients() const;

		/* Rendering of a Primitive3 on a PCLVisualizer. */
		virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const {}

		/* Variable to enable the control over the visualization of primitives. */
		void setVisualizeOnOff(bool);

		/* Render the intersection point p obtained by the cam axis. */
		virtual void setIntersectionPoint(Eigen::Vector3f p) {}

		/* Set the index for a specific face to be highlighted in the viewer. */
		virtual void setFaceHighlight(int i) {}

		virtual void reset();

		/* Receives a PointCloud object by reference and extracts/segments it by fitting to it. */
		virtual void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool seg_normals) {}

		/* Receives a PointCloud and a segmentation object by reference and extracts/segments it by fitting to it. */
		virtual void fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg) {}

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
			float d = 0.0;
			float radius = 0.0;
			float e0_x = 0.0;
			float e0_y = 0.0;
			float e0_z = 0.0;
			float e1_x = 0.0;
			float e1_y = 0.0;
			float e1_z = 0.0;

			Eigen::Vector3f v;
			float angle;
		} properties_;

		pcl::ModelCoefficients::Ptr coefficients_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
		bool isempty_ = true;

		//pcl::ModelCoefficients::Ptr getCoefficients() const;
				
		void fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL);
		void fit_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

		void fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const static int SAC_METHOD, pcl::SacModel SAC_MODEL);
		void fit_sample_consensus_with_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg);

		/* Checks if the fit is valid. */
		virtual bool is_fit_valid() { return false; }

		/* Correct the obtained coefficients if necessary. */
		virtual void correct_coefficients() {}

		/* Update the properties of the Primitive3. */
		virtual void update_properties() {}

		/* Variable to enable the control over the visualization of primitives. */
		bool visualizeOnOff_ = true;

		/**** PCL utils ****/

		/* Calculates the dot product between two PointXYZ objects. */
		float dotPointXYZ(pcl::PointXYZ a, pcl::PointXYZ b);

		/* Calculates the norm of a PointXYZ object. */
		float normPointXYZ(pcl::PointXYZ c);

		/* Calculates the extreme projection values of all PointXYZ in the PointCloud along a given axis. */
		std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ center, pcl::PointXYZ axis);

		/* Calculates the projection extremes of a point cloud about a specific axis 'axis' from point 'center'. */
		std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ center, pcl::PointXYZ axis, pcl::PointXYZ& min, pcl::PointXYZ& max);

		/* Calculates the ranges (min,max) of PointCloud along the x-, y- and z-axis. */
		std::array<float, 6> getPointCloudRanges(const pcl::PointCloud<pcl::PointXYZ>& cloud);
	};

	class Primitive3d1 :
		public Primitive3
	{
		/* Dummy Class */
	};

	class Primitive3d2 :
		public Primitive3
	{
		/* Dummy Class */
	};

	enum class HEURISTIC
	{
		LASER_ARRAY_SINGLE,
		LASER_ARRAY_CROSS,
		LASER_ARRAY_STAR
	};

	class Primitive3d3 :
		public Primitive3
	{
	public:
		/* Dummy Class */

		void addSubPrimitive(Primitive3d1* p);

		/* Reshapes a primitive based on an heuristic. */
		void heuristic(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr, pcl::SACSegmentation<pcl::PointXYZ>* seg, HEURISTIC heu);

	protected:
		std::vector< std::vector<Primitive3d1*>> subprims_;
		bool are_subprims_custom_ = false;

		/* Receives a PointCloud cut by reference and fits a sub-primitive to it. */
		virtual void cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) { std::cout << "DO NOT SHOW THIS MESSAGE" << std::endl; }

		/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
		virtual void cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg) { std::cout << "DO NOT SHOW THIS MESSAGE" << std::endl; }

		bool heuristic_check(HEURISTIC heu);

		void heuristic_prim(HEURISTIC heu);

		virtual void heuristic_laser_array_single() {}
		virtual void heuristic_laser_array_cross() {}
		virtual void heuristic_laser_array_star() {}
	};
}