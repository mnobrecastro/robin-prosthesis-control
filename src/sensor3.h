#pragma once
#include "sensor.h"

#include <iostream>
#include <mutex>
#include <typeinfo>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace robin {

	class Sensor3 :
		public Sensor
	{
	public:
		Sensor3();
		~Sensor3();

		void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();

		void addChild(Sensor3* s);

	protected:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
		std::mutex mu_cloud_;

		Sensor3* parent_ = nullptr;
		std::vector<Sensor3*> children_;

		void feedChildren();
		virtual void fromParent(const pcl::PointCloud<pcl::PointXYZ>& cloud);

	private:
		static int counter;		
	};

	//int Sensor::counter = 0;

}