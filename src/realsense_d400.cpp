#include "realsense_d400.h"

namespace robin {

	RealsenseD400::RealsenseD400()
	{
		_dev = [] {
			rs2::context ctx;
			std::cout << "Waiting for device..." << std::endl;
			while (true) {
				for (auto&& dev : ctx.query_devices())
					return dev;
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}();

		std::cout << "Device found:" << std::endl;
		std::cout << _dev.get_info(RS2_CAMERA_INFO_NAME) << " "
			<< _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
			<< _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;

		std::cout << "A new RealsenseD400 was created\n" << std::endl;
	}

	RealsenseD400::~RealsenseD400() {}

	void RealsenseD400::printInfo() {

		auto sensors = _dev.query_sensors();
		for (rs2::sensor& sensor : sensors) {
			std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
			for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
				if (profile.is<rs2::video_stream_profile>() && profile.stream_name() == "Depth") {
					rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
					std::cout << " Video stream: " << video_stream_profile.format() << " " <<
						video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << "Hz" << std::endl;
				}
				if (profile.is<rs2::motion_stream_profile>() && profile.stream_name() == "Accel") {
					rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
					std::cout << " Motion stream: " << motion_stream_profile.format() << " " <<
						motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << "Hz" << std::endl;
				}
				if (profile.is<rs2::pose_stream_profile>() && profile.stream_name() == "Gyro") {
					rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
					std::cout << " Pose stream: " << pose_stream_profile.format() << " " <<
						pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << "Hz" << std::endl;
				}
				//std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
			}
		}
	}

	void RealsenseD400::start(bool DISPARITY)
	{
		_serialnumber = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		std::cout << "Opening pipeline for " << _serialnumber << std::endl;
		_cfg.enable_device(_serialnumber);
		if (!DISPARITY) {
			//cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90); //works fine!
			_cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
			//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
			////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
		}
		else {
			//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
			_cfg.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
		}

		auto advanced_dev = _dev.as<rs400::advanced_mode>();
		STDepthTableControl depth_table = advanced_dev.get_depth_table();
		if (!DISPARITY) {
			depth_table.depthUnits = 1000; // 0.001m
			depth_table.depthClampMin = 0;
			depth_table.depthClampMax = 200; // mm
			depth_table.disparityShift = 0;
		}
		else {
			depth_table.depthUnits = 1000;  // 0.001m
			depth_table.depthClampMin = 0;
			depth_table.depthClampMax = 200; // mm
			depth_table.disparityShift = 145; // 145@30 or [125-175]@100
		}
		advanced_dev.set_depth_table(depth_table);

		rs2::pipeline_profile profile = _pipe.start(_cfg);
	}

	void RealsenseD400::captureFrame()
	{
		std::cout << "RealsenseD400.captureFrame()" << std::endl;

		std::time_t t0, tf;
		t0 = std::time(0);

		// Wait for the next set of frames from the camera
		rs2::frameset _frames(_pipe.wait_for_frames());
		rs2::depth_frame _depth(_frames.get_depth_frame());

		// Generate the pointcloud and texture mappings
		_points = _pc.calculate(_depth);

		// Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
		this->points_to_pcl();

		tf = std::time(0);
		std::cout << "\nRead pointcloud from " << cloud_->size() << " data points (in " << std::difftime(t0, tf) / 1000 << " ms)." << std::endl;
	}

	void RealsenseD400::points_to_pcl()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		auto sp = _points.get_profile().as<rs2::video_stream_profile>();
		cloud->width = sp.width();
		cloud->height = sp.height();
		cloud->is_dense = false;
		cloud->points.resize(_points.size());
		auto ptr = _points.get_vertices();
		for (auto& p : cloud->points) {
			p.x = ptr->x;
			p.y = ptr->y;
			p.z = ptr->z;
			ptr++;
		}
		cloud_ = cloud;
	}

}