#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_clr(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloud_raw_clr_ = cloud_raw_clr;
	}

	Solver3::~Solver3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPointCloud() const {
		return cloud_;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Solver3::getRawColored() const {
		return cloud_raw_clr_;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPreprocessed() const {
		return cloud_preproc_;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
		return;
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;

		// Initialization of the SACSegmentation objects for each Primitive3
		// (in case the user calls for multiple simultaneous fitting).
		seg_obj_sph_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
		seg_obj_cub_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
		seg_obj_cyl_ = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
	}

	void Solver3::setUseNormals(bool seg_normals)
	{
		seg_normals_ = seg_normals;
		return;
	}

	void Solver3::setPlaneRemoval(bool seg_plane_removal)
	{
		seg_plane_removal_ = seg_plane_removal;
		return;
	}

	std::vector<robin::Sensor3*> Solver3::getSensors() const
	{
		return sensors_;
	}


	void Solver3::setCrop(float x0 = 999.9, float x1 = 999.9, float y0 = 999.9, float y1 = 999.9, float z0 = 999.9, float z1 = 999.9)
	{
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };
		return;
	}

	void Solver3::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
		return;
	}

	void Solver3::setResample(size_t order, float radius)
	{
		resampleOnOff_ = true;
		resamp_order_ = order;
		resamp_radius_ = radius;
		return;
	}

	void Solver3::setPCA(bool b = false) {

	}


	void Solver3::setFairSelection(bool fairness)
	{
		fairselectionOnOff_ = fairness;
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::trimPointCloud()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZ>());
		for (auto p : cloud_->points) {
			if (limits_[0] <= p.x && p.x <= limits_[1] &&
				limits_[2] <= p.y && p.y <= limits_[3] &&
				limits_[4] <= p.z && p.z <= limits_[5]) {
				cloud_trimmed->push_back(pcl::PointXYZ(p.x, p.y, p.z));
			}
		}
		return cloud_trimmed;
	}

	void Solver3::crop()
	{
		if (filterOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Build a passthrough filter to remove unwated points
			pcl::PassThrough<pcl::PointXYZ> pass(true);
			if (false) {
				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("x");
				pass.setFilterLimits(limits_[0], limits_[1]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(limits_[2], limits_[3]);
				pass.filter(*cloud_);

				pass.setInputCloud(cloud_);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(limits_[4], limits_[5]);
				pass.filter(*cloud_);
			}
			else {
				cloud_ = trimPointCloud();
			}

			tf = std::time(0);
			std::cout << "PointCloud after filtering: " << cloud_->points.size() << " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::downsample()
	{
		if (downsampleOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Downsampling the filtered point cloud
			if (true) {
				pcl::ApproximateVoxelGrid<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
				dsfilt.filter(*cloud_);
			}
			else {
				pcl::RandomSample<pcl::PointXYZ> dsfilt;
				dsfilt.setInputCloud(cloud_);
				dsfilt.setSample(2000);
				dsfilt.filter(*cloud_);
			}

			tf = std::time(0);
			std::cerr << "PointCloud after downsampling: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
				<< " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::resample()
	{
		if (resampleOnOff_ && cloud_->points.size() > 0) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Smoothing a raw point cloud by resampling it

			// Create a kD-Tree for point search
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

			// Output has the PointNormal type in order to store the normals calculated by MLS
			pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

			// Init object (second point type is for the normals, even if unused)
			pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

			mls.setComputeNormals(true);

			// Set MovingLeastSquares parameters
			mls.setInputCloud(cloud_);
			mls.setPolynomialOrder(resamp_order_);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(resamp_radius_);

			// Reconstruct
			mls.process(*mls_points);

			// Copying the PointCloud
			cloud_->clear();
			cloud_->resize(mls_points->size());
			auto ptr = mls_points->points.begin();
			for (auto& p : cloud_->points) {
				p.x = ptr->x;
				p.y = ptr->y;
				p.z = ptr->z;
				ptr++;
			}

			tf = std::time(0);
			std::cerr << "PointCloud after resampling: " << cloud_->width * cloud_->height << "=" << cloud_->points.size()
				<< " data points (in " << std::difftime(t0, tf) << " ms).\n";
		}
		return;
	}

	void Solver3::principal_components(robin::Primitive3d3*& prim)
	{
		primitive_ = prim;

		// Reset the solver's (temp and coloured) PointClouds
		cloud_->clear();
		cloud_raw_clr_->clear();

		for (Sensor3* s : sensors_) {
			*cloud_ += *s->getPointCloud();
			*cloud_raw_clr_ += *s->getRawColored();
		}

		this->crop();
		this->downsample();
		//this->resample();

		// Store the pre-processed cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to compute the Principal Components.\n";
			return;
		}

		primitive_->pca(*cloud_);

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		//prim = primitive_;
		return;
	}

	void Solver3::solve(robin::Primitive3d3*& prim)
	{
		primitive_ = prim;

		// Reset the solver's (temp and coloured) PointClouds
		cloud_->clear();
		cloud_raw_clr_->clear();

		for (Sensor3* s : sensors_) {
			*cloud_ += *s->getPointCloud();
			*cloud_raw_clr_ += *s->getRawColored();
		}

		this->crop();
		this->downsample();
		//this->resample();

		// Store the pre-processed cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		this->segment();

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		prim = primitive_;
		return;
	}

	void Solver3::segment()
	{
		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform SAC segmentation.\n";
			return;
		}
		
		// Check whether any Primitive3 has been provided
		if (primitive_ != nullptr) {
			// Perform initial plane removal
			if (seg_plane_removal_) {
				robin::Primitive3Plane plane;
				if (seg_obj_ptr_ != nullptr) {					
					plane.fit(cloud_, seg_obj_ptr_);
				}
				else{
					plane.fit(cloud_, seg_normals_);
				}
			}

			// Check whether a specific/non-generice Primitive3 has been provided
			if (typeid(*primitive_) != typeid(robin::Primitive3d3)) {
				this->fitPrimitive(primitive_, cloud_, seg_obj_ptr_);
			}
			else {
				// Multiple simultaneous fitting of pre-defined Primtive3's 

				// Set the coefficients for each Primitive3 (i.e. initial guess)
				p_sph_->setCoefficients({ 0.000, 0.000, 0.150, 0.050 });
				//p_cub_->setCoefficients({ 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000 }); // N/A
				p_cyl_->setCoefficients({ 0.000, 0.000, 0.150, 0.000, 0.000, 0.100, 0.050 });

				// Define a copy of the PointCloud for each Primitive3				
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_sph(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cub(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cyl(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));

				// Fit each primitive to its respective PointCloud copy
#ifdef MULTITHREADING
				std::thread t_sph = std::thread(&Solver3::fitPrimitive, this, std::ref(p_sph_), std::ref(c_sph), std::ref(seg_obj_sph_));
				std::thread t_cub = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cub_), std::ref(c_cub), std::ref(seg_obj_cub_));
				std::thread t_cyl = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cyl_), std::ref(c_cyl), std::ref(seg_obj_cyl_));
				t_sph.join();
				t_cub.join();
				t_cyl.join();
#else
				p_sph_->fit(c_sph, seg_obj_sph_);
				p_cub_->fit(c_cub, seg_obj_cub_);
				p_cyl_->fit(c_cyl, seg_obj_cyl_);
#endif

				// Selection of the best fitting Primitive3d3
				float cloud_size(cloud_->points.size());
				float fit_percent(0.0);

				float sph_size(p_sph_->getPointCloud()->points.size());
				std::cout << "Sphere FitPercent: " << sph_size / cloud_size << '\n';
				float cub_size(p_cub_->getPointCloud()->points.size());
				std::cout << "Cuboid FitPercent: " << cub_size / cloud_size << '\n';
				float cyl_size(p_cyl_->getPointCloud()->points.size());
				std::cout << "Cylinder FitPercent: " << cyl_size / cloud_size << '\n';
				
				if (fairselectionOnOff_) {
					if (sph_size == 0 || cub_size == 0 || cyl_size == 0) {
						// Fair selection: if at least one of the Primitive3d3 has not been fitted, then no primitive should be selected.
						primitive_ = primitive_;
						pcl::PointCloud<pcl::PointXYZ>::Ptr c_empty(new pcl::PointCloud<pcl::PointXYZ>());
						cloud_ = c_empty;
						return;
					}
				}
				// Pick the biggest fit cloud that corresponds to the correct primitive fitting
				if (fit_percent < sph_size / cloud_size) {
					fit_percent = sph_size / cloud_size;
					primitive_ = p_sph_;
					cloud_ = c_sph;
				}
				if (fit_percent < cub_size / cloud_size) {
					fit_percent = cub_size / cloud_size;
					primitive_ = p_cub_;
					cloud_ = c_cub;
				}
				if (fit_percent < cyl_size / cloud_size) {
					fit_percent = cyl_size / cloud_size;
					primitive_ = p_cyl_;
					cloud_ = c_cyl;
				}
			}
		}
		return;
	}

	void Solver3::fitPrimitive(robin::Primitive3d3*& prim, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj)
	{
		// Check whether an instance of Segmentation has been provided.
		if (seg_obj_ptr_ != nullptr) {
			std::cout << "Segmentation object has been provided!\n";
			prim->fit(cloud, seg_obj);
		}
		else {
			std::cout << "NO segmentation object has been provided.\n";
			prim->fit(cloud, seg_normals_);
		}
	}
}