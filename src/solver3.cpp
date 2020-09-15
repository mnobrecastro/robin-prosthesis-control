#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
	}

	Solver3::~Solver3() {}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPointCloud() const {
		return cloud_;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Solver3::getPreprocessed() const {
		return cloud_preproc_;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;
	}

	void Solver3::setUseNormals(bool seg_normals)
	{
		seg_normals_ = seg_normals;
	}

	void Solver3::setPlaneRemoval(bool seg_plane_removal)
	{
		seg_plane_removal_ = seg_plane_removal;
	}

	std::vector<robin::Sensor3*> Solver3::getSensors() const
	{
		return sensors_;
	}


	void Solver3::setCrop(float x0 = 999.9, float x1 = 999.9, float y0 = 999.9, float y1 = 999.9, float z0 = 999.9, float z1 = 999.9)
	{
		filterOnOff_ = true;
		limits_ = { x0, x1, y0, y1, z0, z1 };
	}

	void Solver3::setDownsample(float voxel_size = 0.005)
	{
		downsampleOnOff_ = true;
		voxel_size_ = voxel_size;
	}

	void Solver3::setResample(size_t order, float radius)
	{
		resampleOnOff_ = true;
		resamp_order_ = order;
		resamp_radius_ = radius;
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
		if (filterOnOff_) {
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
			std::cout << "PointCloud after filtering: " << cloud_->points.size() << " data points (in " << std::difftime(t0, tf) << " ms)." << std::endl;
		}
	}

	void Solver3::downsample()
	{
		if (downsampleOnOff_) {
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
				<< " data points (in " << std::difftime(t0, tf) << " ms)." << std::endl;
		}
	}

	void Solver3::resample()
	{
		if (resampleOnOff_) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Smoothing a raw point cloud by resampling it

			// Create a kD-Tree for point search
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

			// Output has the PointNormal type in order to store the normals calculated by MLS
			pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);

			// Init object (second point type is for the normals, even if unused)
			pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
			//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

			mls.setComputeNormals(true);
			//mls.setComputeNormals(false);

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
				<< " data points (in " << std::difftime(t0, tf) << " ms)." << std::endl;
		}
	}

	void Solver3::solve(robin::Primitive3d3*& prim)
	{
		primitive_ = prim;

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		for (Sensor3* s : sensors_) {
			*cloud_ += *s->getPointCloud();
		}

		this->crop();
		this->downsample();
		//this->resample();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
		cloud_preproc_ = cloud_preproc;

		this->segment();

		// Copy the address in case a Primitive3d3 has been provided and mutated to a derived primitive
		prim = primitive_;
	}

	void Solver3::segment()
	{
		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			primitive_->reset();
			std::cerr << "* Not enough points to perform SAC segmentation." << std::endl;
			return;
		}
		
		// Check whether a Primitive3 has been provided
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

			if (typeid(*primitive_) != typeid(robin::Primitive3d3)) {
				this->fitPrimitive(primitive_, cloud_, seg_obj_ptr_);
			}
			else {
				// Erase the dummy/generic Primitive3d3
				//delete primitive_;

				robin::Primitive3d3* p_sph = new robin::Primitive3Sphere;
				robin::Primitive3d3* p_cub = new robin::Primitive3Cuboid;
				robin::Primitive3d3* p_cyl = new robin::Primitive3Cylinder;

				pcl::PointCloud<pcl::PointXYZ>::Ptr c_sph(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cub(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
				pcl::PointCloud<pcl::PointXYZ>::Ptr c_cyl(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));

				pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_sph = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
				pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_cub = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);
				pcl::SACSegmentation<pcl::PointXYZ>* seg_obj_cyl = new pcl::SACSegmentation<pcl::PointXYZ>(*seg_obj_ptr_);

				std::thread t_sph = std::thread(&Solver3::fitPrimitive, this, std::ref(p_sph), std::ref(c_sph), std::ref(seg_obj_sph));
				std::thread t_cub = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cub), std::ref(c_cub), std::ref(seg_obj_cub));
				std::thread t_cyl = std::thread(&Solver3::fitPrimitive, this, std::ref(p_cyl), std::ref(c_cyl), std::ref(seg_obj_cyl));

				t_sph.join();
				t_cub.join();
				t_cyl.join();

				// Selection of the best fitting Primitive3d3
				float cloud_size(cloud_->points.size());
				float fit_percent(0.0);

				float sph_size(p_sph->getPointCloud()->points.size());
				std::cout << "Sphere FitPercent: " << sph_size / cloud_size << std::endl;
				float cub_size(p_cub->getPointCloud()->points.size());
				std::cout << "Cuboid FitPercent: " << cub_size / cloud_size << std::endl;
				float cyl_size(p_cyl->getPointCloud()->points.size());
				std::cout << "Cylinder FitPercent: " << cyl_size / cloud_size << std::endl;
				
				if (sph_size == 0 || cub_size == 0 || cyl_size == 0) {
					// Fair selection: if at least one of the Primitive3d3 has not been fitted, then no primitive should be selected.
					primitive_ = primitive_;
					pcl::PointCloud<pcl::PointXYZ>::Ptr c_empty (new pcl::PointCloud<pcl::PointXYZ>());
					cloud_ = c_empty;
				}
				else {
					// Pick the biggest fit cloud that corresponds to the correct primitive fitting
					if (fit_percent < sph_size / cloud_size) {
						fit_percent = sph_size / cloud_size;
						primitive_ = p_sph;
						cloud_ = c_sph;
					}
					if (fit_percent < cub_size / cloud_size) {
						fit_percent = cub_size / cloud_size;
						primitive_ = p_cub;
						cloud_ = c_cyl;
					}
					if (fit_percent < cyl_size / cloud_size) {
						fit_percent = cyl_size / cloud_size;
						primitive_ = p_cyl;
						cloud_ = c_cyl;
					}
				}
			}
		}
	}

	void Solver3::fitPrimitive(robin::Primitive3d3*& prim, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::SACSegmentation<pcl::PointXYZ>*& seg_obj)
	{
		// Check whether an instance of Segmentation has been provided.
		if (seg_obj_ptr_ != nullptr) {
			std::cout << "Segmentation object has been provided!" << std::endl;
			prim->fit(cloud, seg_obj);
		}
		else {
			std::cout << "NO segmentation object has been provided." << std::endl;
			prim->fit(cloud, seg_normals_);
		}
	}

	void Solver3::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		/* Not implemented yet. */
	}
}