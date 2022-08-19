#pragma once
#include "solver3.h"
#include "robin/sensor/sensor3.h"
#include "robin/sensor/laser_array.h"
#include "robin/primitive/primitive3.h"

#include <typeinfo>

extern std::chrono::time_point<std::chrono::steady_clock> tic;

namespace robin
{
	class Solver3Lasers :
		public Solver3
	{
	public:
		Solver3Lasers() {}
		~Solver3Lasers() {}

		void addSensor(robin::Sensor3Array*);

		//void solve(robin::Primitive3& prim) = delete;
		void solve(robin::Primitive3d3*& prim);

		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, std::string draw = "wireframe") const;

	protected:
		Primitive3d3* primitive_ = nullptr;
		HEURISTIC heu_;

		unsigned int MIN_POINTS_PROCEED_ = 10;

		/* Remove the method from the interface. */
		using Solver3::addSensor;

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr_;

		/* Performs heuristic fitting of the primitive. */
		void heuristic();

		/* Auxiliary Primitive heuristic function for multithreading. */
		void heuPrimitive(robin::Primitive3d3*& prim, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj, HEURISTIC heu);

	};
}