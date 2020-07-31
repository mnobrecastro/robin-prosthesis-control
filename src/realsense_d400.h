#pragma once
#include "camera_depth.h"

#include <iostream>
#include <thread>
#include <ctime>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

namespace robin
{
	class RealsenseD400 :
		public CameraDepth
	{
	public:
		RealsenseD400();
		~RealsenseD400();

		void printInfo();

		void setDisparity(bool disparity=false);

		void start(bool);// override;

		void captureFrame();

	protected:
		void points_to_pcl(rs2::points pts);

	private:

		rs2::device dev_;
		bool DISPARITY_ = false;

		// RealSense2 pipeline objects and pointcloud, points.
		rs2::pipeline pipe_;
		rs2::config cfg_;
		std::string serialnumber_;
	};
}