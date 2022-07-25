#pragma once
#include "camera_rgb.h"

#include <array>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

namespace robin {

	/*enum class CAMERA {
		REALSENSE_D400
	};

	class RgbdMask :
		public CameraRgb
	{
	public:
		
		RgbdMask() = delete;
		RgbdMask(robin::CAMERA);
		~RgbdMask();

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
	};*/

}