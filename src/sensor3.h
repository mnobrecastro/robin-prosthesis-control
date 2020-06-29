#pragma once
#include "sensor.h"

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robin {

	class Sensor3 :
		public Sensor
	{
	public:
		Sensor3();
		~Sensor3();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

	protected:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

	private:
		static int counter;
	};

	//int Sensor::counter = 0;

}