#include "solver3_lasers.h"

namespace robin
{
	void Solver3Lasers::addSensor(Sensor3Array* s_arr)
	{
		if (typeid(*s_arr) == typeid(robin::LaserArraySingle)) {
			heu_ = HEURISTIC::LASER_ARRAY_SINGLE;
		}
		else if (typeid(*s_arr) == typeid(robin::LaserArrayCross)) {
			heu_ = HEURISTIC::LASER_ARRAY_CROSS;
		}
		else if (typeid(*s_arr) == typeid(robin::LaserArrayStar)) {
			heu_ = HEURISTIC::LASER_ARRAY_STAR;
		}

		for (auto s : s_arr->getSensors()) {
			sensors_.push_back(s);
		}
	}

	void Solver3Lasers::solve(robin::Primitive3d3& prim)
	{
		primitive_ = &prim;

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr;
		bool has_invalid_cloud(false);

		for (Sensor3* s : sensors_) {
			cloud_ = s->getPointCloud();
			this->crop();
			this->downsample();

			// Check whether the PointCloud has enough points to proceed
			if (cloud_->points.size() < MIN_POINTS_PROCEED_) { has_invalid_cloud = true; }

			cloud_arr.push_back(cloud_);
			*cloud_temp += *cloud_;
		}

		cloud_ = cloud_temp;

		// Check whether the PointCloud has enough points to proceed
		if (has_invalid_cloud) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform the heuristic fitting." << std::endl;
			return;
		}

		primitive_->heuristic(cloud_arr, seg_obj_ptr_, heu_);
	}
		
}