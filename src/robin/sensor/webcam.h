#pragma once
#include "camera_rgb.h"

#include <iostream>
#include <string>
#include <ctime>
#include <thread>

#include <opencv2/videoio.hpp>

namespace robin
{
	class Webcam :
		public CameraRgb
	{
	public:
		Webcam() = delete;
		Webcam(int dev_index);
		Webcam(const std::string dev_str);
		~Webcam();

		void printInfo();

	protected:
		//std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
		//void points_to_pcl(const rs2::points pts, const rs2::video_frame color);
		void captureFrame();

	private:

		std::shared_ptr<cv::VideoCapture> dev_;

		//cv::VideoCapture dev_;
		bool DISPARITY_ = false;

		// RealSense2 pipeline objects and pointcloud, points.
		//rs2::pipeline pipe_;
		//rs2::config cfg_;
		std::string serialnumber_;
	};
		
}