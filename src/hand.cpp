#include "hand.hpp"

#include <iostream>
#include <windows.h>

namespace robin
{	
	Hand::Hand()
	{
		viewer_ = initVisualizer();
	}

	Hand::~Hand() {}

	pcl::visualization::PCLVisualizer::Ptr Hand::initVisualizer()
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

	void Hand::addSensor(robin::Sensor* sensor)
	{		
		_sensors.push_back(sensor);
	}

	void Hand::addPrimitive(robin::Primitive& prim)
	{
		_primitives.push_back(prim);
	}

	void Hand::startSensors(bool disparityOnOff)
	{
		for (auto sensor : _sensors) {
			(*sensor).start(disparityOnOff);
		}
	}

	void Hand::activate()
	{
		Hand::startSensors(false);

		while (true) { //!Hand::iskeypressed(1)

			for (auto s : _sensors) {
				(*s).captureFrame();
			}

			// Clean existing point clouds
			viewer_->removeAllShapes();
			viewer_->removeAllPointClouds();

			for (auto s : _sensors) {
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>((*s).getPointCloud()));
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h((int) 255 * 1.0, (int)255 * 0.0, (int)255 * 0.0);
				cloud_color_h.setInputCloud(cloud);
				viewer_->addPointCloud(cloud, cloud_color_h);			
			}
			viewer_->spinOnce(100, true);
		}
	}

	bool Hand::iskeypressed(unsigned timeout_ms = 0)
	{
		return WaitForSingleObject(
			GetStdHandle(STD_INPUT_HANDLE),
			timeout_ms
		) == WAIT_OBJECT_0;
	}
}