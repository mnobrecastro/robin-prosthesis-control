#pragma once
#include "sensor2.h"

#include <iostream>
#include <ctime>
#include <array>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

namespace robin {

	enum class CAMERA {
		REALSENSE_D400
	};

	class CameraRgb :
		public Sensor2
	{
	public:
		
		CameraRgb();
		CameraRgb(robin::CAMERA);
		~CameraRgb();

		virtual void printInfo();

		virtual void captureFrame();

		void setCrop(float, float, float, float, float, float);
		void setDownsample(float);

	protected:

		void crop();
		void downsample();

		std::shared_ptr<cv::Mat> trimImage();

		bool filterOnOff_ = false;
		std::array<float, 6> limits_;
		bool downsampleOnOff_ = false;
		float voxel_size_ = 0.005f;
	};

}