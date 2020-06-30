#include "solver3.h"

namespace robin
{
	Solver3::Solver3()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud_ = cloud;

		viewer_ = initVisualizer();
	}

	/*Solver3::Solver3(robin::Primitive3*)
	{
		Solver3();
		//
	}*/

	Solver3::~Solver3() {}

	pcl::visualization::PCLVisualizer::Ptr Solver3::initVisualizer()
	{
		//  Visualiser initiallization
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		int vp(0); // Default viewport
		viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
		viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
		viewer->setSize(800, 600);
		float bckgr_gray_level = 0.0;  // Black:=0.0
		float txt_gray_lvl = 1.0 - bckgr_gray_level;
		viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
		viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)
		viewer->addText("Some text here\nSome text there", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "text", vp);
		return viewer;
	}

	void Solver3::addSensor(robin::Sensor3* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver3::setPrimitive(robin::Primitive3& prim)
	{
		primitive_ = &prim;
	}


	void Solver3::setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj)
	{
		seg_obj_ = seg_obj;
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



	void Solver3::startSensors(bool disparityOnOff)
	{
		for (auto s : sensors_) {
			s->start(disparityOnOff);
		}
	}

	void Solver3::activate()
	{
		Solver3::startSensors(false);

		while (true) { //!Hand::iskeypressed(1)			

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
				continue;
			}

			// Clean any previously rendered objects
			viewer_->removeAllShapes();
			viewer_->removeAllPointClouds();

			for (Sensor3* s : sensors_) {
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h((int)255 * 1.0, (int)255 * 1.0, (int)255 * 1.0);
				cloud_color_h.setInputCloud(s->getPointCloud());
				viewer_->addPointCloud(s->getPointCloud(), cloud_color_h);
			}

			this->segment();

			viewer_->spinOnce(1, true);
		}
	}

	void Solver3::segment()
	{
		switch (seg_method_) {
		case Method3::SEGMENTATION_RANSAC:
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
				if (seg_obj_ != nullptr) {					
					plane.fit(cloud_, seg_obj_);
				}
				else{
					plane.fit(cloud_, seg_normals_);
				}
				// (temporary plot)
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color_h(0, 255, 0);
				plane_color_h.setInputCloud(plane.getPointCloud());
				viewer_->addPointCloud(plane.getPointCloud(), plane_color_h, "plane");
				plane.visualize(viewer_);
			}

			// Check whether an instance of Segmentation has been provided.
			if (seg_obj_ != nullptr) {
				primitive_->fit(cloud_, seg_obj_);
				std::cout << "Segmentation object has been provided!" << std::endl;
			}
			else {
				primitive_->fit(cloud_, seg_normals_);
				std::cout << "NO segmentation object has been provided." << std::endl;
			}
			// (temporary plot)
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
			primitive_color_h.setInputCloud(primitive_->getPointCloud());
			viewer_->addPointCloud(primitive_->getPointCloud(), primitive_color_h, "primitive");
			primitive_->visualize(viewer_);
		}
	}

	void Solver3::segmentLCCP()
	{
		/* Not yet implemented. */
	}
}