#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;
	}

	Solver3::~Solver3() {}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ptr_ = seg_obj;
	}

	void Solver3::setSegmentation(robin::Method3 seg_method)
	{
		seg_method_ = seg_method;
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

	void Solver3::solve(robin::Primitive3& prim)
	{
		primitive_ = &prim;	

		// Reset the solver's (temp) PointCloud 
		cloud_->clear();

		for (Sensor3* s : sensors_) {
			s->captureFrame();
			s->crop();
			s->downsample();
			*cloud_ += *s->getPointCloud();
		}

		// Check whether the PointCloud has enough points to proceed
		if (cloud_->points.size() < MIN_POINTS_PROCEED_) {
			std::cerr << "* Not enough points to perform segmentation." << std::endl;
			//std::cout << "** Total elapsed time: " << tloop.toc() << " ms." << std::endl;
			return;
		}

		this->segment();
	}

	void Solver3::segment()
	{
		switch (seg_method_) {
		case Method3::SEGMENTATION_SAC:
			this->segmentSAC();
			break;
		case Method3::SEGMENTATION_LCCP:
			this->segmentLCCP();
			break;
		default:
			std::cerr << "Invalid Method3!" << std::endl;
			return;
		}
	}

	void Solver3::segmentSAC()
	{
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
				// (temporary plot)
				/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color_h(0, 255, 0);
				plane_color_h.setInputCloud(plane.getPointCloud());
				viewer_->addPointCloud(plane.getPointCloud(), plane_color_h, "plane");
				plane.visualize(viewer_);*/
			}

			// Check whether an instance of Segmentation has been provided.
			if (seg_obj_ptr_ != nullptr) {
				std::cout << "Segmentation object has been provided!" << std::endl;
				primitive_->fit(cloud_, seg_obj_ptr_);				
			}
			else {
				std::cout << "NO segmentation object has been provided." << std::endl;
				primitive_->fit(cloud_, seg_normals_);				
			}
			// (temporary plot)
			/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
			primitive_color_h.setInputCloud(primitive_->getPointCloud());
			viewer_->addPointCloud(primitive_->getPointCloud(), primitive_color_h, "primitive");
			primitive_->visualize(viewer_);*/
		}
	}

	void Solver3::segmentLCCP()
	{
		/* Not yet implemented. */
	}
}