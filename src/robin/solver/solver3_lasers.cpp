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

	void Solver3Lasers::solve(robin::Primitive3d3*& prim)
	{
		primitive_ = prim;

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		// Reset the solver's (temp) cloud_arr
		cloud_arr_.clear(); //shared_ptr handles memory

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
		bool has_invalid_cloud(false);

		for (Sensor3* s : sensors_) {
			cloud_ = s->getPointCloud();
			this->crop();
			this->downsample();

			// Check whether the PointCloud has enough points to proceed
			if (cloud_->points.size() < MIN_POINTS_PROCEED_) { has_invalid_cloud = true; }

			cloud_arr_.push_back(cloud_);
			*cloud_temp += *cloud_;
		}

		// Store the pre-processed cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		cloud_ = cloud_temp; //(Redundant)

		// Check whether the PointCloud has enough points to proceed
		// (tiggered if one of the lasers is empty)
		if (has_invalid_cloud) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform the heuristic fitting." << std::endl;
			return;
		}

		this->heuristic();

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		prim = primitive_;
	}

	void Solver3Lasers::heuristic()
	{
		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform the Heuristic fitting." << std::endl;
			return;
		}

		// Check whether any Primitive3 has been provided
		if (primitive_ != nullptr) {
			// Check whether a specific/non-generice Primitive3 has been provided
			if (typeid(*primitive_) != typeid(robin::Primitive3d3)) {
				this->heuPrimitive(primitive_, cloud_arr_, seg_obj_ptr_, heu_);
			}
			else {
				// Multiple simultaneous fitting of pre-defined Primtive3's 

				// Set the coefficients for each Primitive3d3 (i.e. initial guess)
				p_sph_->setCoefficients({ 0.000, 0.000, 0.150, 0.050 });
				//p_cub_->setCoefficients({ 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000 }); // N/A
				p_cyl_->setCoefficients({ 0.000, 0.000, 0.150, 0.000, 0.000, 0.100, 0.050 });

				// Define a copy of the PointCloud array for each Primitive3d3				
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> c_sph;
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> c_cub;
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> c_cyl;
				for (auto cloud_ptr : cloud_arr_) {
					c_sph.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_ptr)));
					c_cub.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_ptr)));
					c_cyl.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_ptr)));
				}

				// Perform the heuristic to each primitive to its respective PointCloud_array copy
#ifdef MULTITHREADING
				std::thread t_sph = std::thread(&Solver3Lasers::heuPrimitive, this, std::ref(p_sph_), std::ref(c_sph), std::ref(seg_obj_sph_), std::ref(heu_));
				std::thread t_cub = std::thread(&Solver3Lasers::heuPrimitive, this, std::ref(p_cub_), std::ref(c_cub), std::ref(seg_obj_cub_), std::ref(heu_));
				std::thread t_cyl = std::thread(&Solver3Lasers::heuPrimitive, this, std::ref(p_cyl_), std::ref(c_cyl), std::ref(seg_obj_cyl_), std::ref(heu_));
				t_sph.join();
				t_cub.join();
				t_cyl.join();
#else
				p_sph_->heuristic(c_sph, seg_obj_ptr_, heu);
				p_cub_->heuristic(c_cub, seg_obj_ptr_, heu);
				p_cyl_->heuristic(c_cyl, seg_obj_ptr_, heu);
#endif

				// Selection of the best fitting Primitive3d3
				float cloud_size(cloud_->points.size());
				float fit_percent(0.0);

				float sph_size(p_sph_->getPointCloud()->points.size());
				std::cout << "Sphere FitPercent: " << sph_size / cloud_size << std::endl;
				float cub_size(p_cub_->getPointCloud()->points.size());
				std::cout << "Cuboid FitPercent: " << cub_size / cloud_size << std::endl;
				float cyl_size(p_cyl_->getPointCloud()->points.size());
				std::cout << "Cylinder FitPercent: " << cyl_size / cloud_size << std::endl;

				if (fairselectionOnOff_) {
					if (sph_size == 0 || cub_size == 0 || cyl_size == 0) {
						// Fair selection: if at least one of the Primitive3d3 has not been fitted, then no primitive should be selected.
						primitive_ = primitive_;
						pcl::PointCloud<pcl::PointXYZ>::Ptr c_empty(new pcl::PointCloud<pcl::PointXYZ>());
						cloud_ = c_empty;
						return;
					}
				}
				// Pick the biggest fit cloud that corresponds to the correct primitive heuristic
				if (fit_percent < sph_size / cloud_size) {
					fit_percent = sph_size / cloud_size;
					primitive_ = p_sph_;
					//cloud_ = c_sph;
				}
				if (fit_percent < cub_size / cloud_size) {
					fit_percent = cub_size / cloud_size;
					primitive_ = p_cub_;
					//cloud_ = c_cub;
				}
				if (fit_percent < cyl_size / cloud_size) {
					fit_percent = cyl_size / cloud_size;
					primitive_ = p_cyl_;
					//cloud_ = c_cyl;
				}
			}
		}

		return;
	}

	void Solver3Lasers::heuPrimitive(robin::Primitive3d3*& prim, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_arr, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj, HEURISTIC heu)
	{
		// Check whether an instance of Segmentation has been provided.
		if (seg_obj_ptr_ != nullptr) {
			std::cout << "Segmentation object has been provided!" << std::endl;
			prim->heuristic(cloud_arr, seg_obj, heu);
		}
		else {
			std::cout << "NO segmentation object has been provided." << std::endl;
			//primitive_->heuristic(cloud_arr, seg_obj_ptr_, heu); // Needs workaround later
		}
	}
}