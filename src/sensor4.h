#pragma once

#include "sensor.h"

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robin {

	class Sensor4 :
		public Sensor
	{
	public:
		Sensor4();
		~Sensor4();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() const;

	protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

	private:
		static int counter;
	};

	//int Sensor::counter = 0;

}