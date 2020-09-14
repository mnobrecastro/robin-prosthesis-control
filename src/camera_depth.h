#pragma once
#include "sensor3.h"

#include <array>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robin {

	enum class CAMERA {
		REALSENSE_D400
	};

	class CameraDepth :
		public Sensor3
	{
	public:
		
		CameraDepth();
		CameraDepth(robin::CAMERA);
		~CameraDepth();

		virtual void printInfo();

		virtual void captureFrame();

		void setCrop(float, float, float, float, float, float);
		void setDownsample(float);

	protected:

		void crop();
		void downsample();

		pcl::PointCloud<pcl::PointXYZ>::Ptr trimPointCloud();

		bool filterOnOff_ = false;
		std::array<float, 6> limits_;
		bool downsampleOnOff_ = false;
		float voxel_size_ = 0.005f;
	};

}