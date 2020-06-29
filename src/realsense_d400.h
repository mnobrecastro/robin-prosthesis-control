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

		void start(bool);// override;

		void captureFrame();

	protected:
		void points_to_pcl();

	private:

		rs2::device _dev;

		// RealSense2 pointcloud, points and pipeline objects
		rs2::pointcloud _pc;
		rs2::points _points;

		// Wait for the next set of frames from the camera
		rs2::pipeline _pipe;
		rs2::config _cfg;
		std::string _serialnumber;
	};
}