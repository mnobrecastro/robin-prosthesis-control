#include "realsense_d400.h"

namespace robin {

	RealsenseD400::RealsenseD400()
	{		
		dev_ = [] {
			rs2::context ctx;
			std::cout << "Waiting for device..." << std::endl;
			while (true) {
				for (auto&& dev : ctx.query_devices())
					return dev;
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}();

		std::cout << "Device found:" << std::endl;
		std::cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;

		std::cout << "A new RealsenseD400 was created\n" << std::endl;

		this->start(DISPARITY_);

		thread_capture_ = std::thread(&RealsenseD400::captureFrame, this);
	}

	RealsenseD400::~RealsenseD400() {}

	void RealsenseD400::printInfo() {

		auto sensors = dev_.query_sensors();
		for (rs2::sensor& sensor : sensors) {
			std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
			for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
				if (profile.is<rs2::video_stream_profile>()) {
					rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
					std::cout << " stream " << video_stream_profile.stream_name() << " " << video_stream_profile.format() << " " <<
						video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << " Hz" << std::endl;
				}
				if (profile.is<rs2::motion_stream_profile>()) {
					rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
					std::cout << " stream " << motion_stream_profile.stream_name() << " " << motion_stream_profile.format() << " " <<
						motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << " Hz" << std::endl;
				}
				if (profile.is<rs2::pose_stream_profile>()) {
					rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
					std::cout << " stream " << pose_stream_profile.stream_name() << " " << pose_stream_profile.format() << " " <<
						pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << " Hz" << std::endl;
				}
				//std::cout << "  stream " << profile.stream_type() << " " << profile.stream_name() << " " << profile.format() << " @" << profile.fps() << " Hz" << std::endl;
			}
		}
		std::cout << std::endl;
	}

	void RealsenseD400::setDisparity(bool disparity)
	{
		DISPARITY_ = disparity;
	}

	void RealsenseD400::start(bool disparity)
	{
		serialnumber_ = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		std::cout << "Opening pipeline for " << serialnumber_ << std::endl;
		cfg_.enable_device(serialnumber_);
		if (!disparity) {
			// Depth stream
			//cfg_.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90); //works fine!
			cfg_.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
			//cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);

			// Color stream
			cfg_.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 60);
		}
		else {
			//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
			cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
		}

		auto advanceddev_ = dev_.as<rs400::advanced_mode>();
		STDepthTableControl depth_table = advanceddev_.get_depth_table();
		if (!disparity) {
			depth_table.depthUnits = 1000; // 0.001m
			depth_table.depthClampMin = 0;
			depth_table.depthClampMax = 300; // mm
			depth_table.disparityShift = 0;
		}
		else {
			depth_table.depthUnits = 1000;  // 0.001m
			depth_table.depthClampMin = 0;
			depth_table.depthClampMax = 200; // mm
			depth_table.disparityShift = 145; // 145@30 or [125-175]@100
		}
		advanceddev_.set_depth_table(depth_table);

		rs2::pipeline_profile profile = pipe_.start(cfg_);
	}

	void RealsenseD400::captureFrame()
	{
		while (true) {
			//std::cout << "RealsenseD400.captureFrame()" << std::endl;

			std::time_t t0, tf;
			t0 = std::time(0);

			// Wait for the next set of frames from the camera
			rs2::frameset frames(pipe_.wait_for_frames());
			rs2::depth_frame depth(frames.get_depth_frame());
			rs2::video_frame color(frames.get_color_frame());

			// Map Color texture to each point
			

			// Generate the pointcloud and color texture mappings
			rs2::pointcloud pc;
			pc.map_to(color); // RGB texture
			rs2::points pts = pc.calculate(depth);

			// Transform rs2::pointcloud into pcl::PointCloud<PointXYZ>::Ptr
			this->points_to_pcl(pts);

			// Feed the children Sensors
			this->feedChildren();

			tf = std::time(0);
			//std::cout << "Read pointcloud from " << cloud_->size() << " data points (in " << std::difftime(t0, tf) / 1000 << " ms)." << std::endl;
		}
	}

	void RealsenseD400::points_to_pcl(rs2::points pts)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		auto sp = pts.get_profile().as<rs2::video_stream_profile>();
		cloud->width = sp.width();
		cloud->height = sp.height();
		cloud->is_dense = false;
		cloud->points.resize(pts.size());
		auto ptr = pts.get_vertices();
		for (auto& p : cloud->points) {
			p.x = ptr->x;
			p.y = ptr->y;
			p.z = ptr->z;
			ptr++;
		}
		mu_cloud_.lock();		
		cloud_ = cloud;
		mu_cloud_.unlock();
	}
}