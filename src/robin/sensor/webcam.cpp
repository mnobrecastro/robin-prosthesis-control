#include "webcam.h"

namespace robin {

	Webcam::Webcam(int dev_index)
	{


		thread_capture_ = std::thread(&Webcam::captureFrame, this);
	}

	Webcam::Webcam(const std::string dev_str)
	{
		// CV_WRAP explicit VideoCapture(const String& filename, int apiPreference, const std::vector<int>& params);

		VCap cap;
		if (!cap.dev_.isOpened()) {
			std::cerr << "ERROR: Could not open the webcam.\n";
		}

		cap.dev_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
		//cam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2'));
		cap.dev_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
		cap.dev_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.dev_.set(cv::CAP_PROP_FPS, 30);

		std::cout << "Device found:\n";
		/*std::cout << dev_.get(RS2_CAMERA_INFO_NAME) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << "\n\n";*/
		std::cout << cap.dev_.get(cv::CAP_PROP_FOURCC) << " "
			<< cap.dev_.get(cv::CAP_PROP_FRAME_WIDTH) << " "
			<< cap.dev_.get(cv::CAP_PROP_FRAME_HEIGHT) << " "
			<< cap.dev_.get(cv::CAP_PROP_FPS) << "\n\n";

		std::cout << "A new Webcam was created.\n" << std::endl;

		//this->start(DISPARITY_);

		thread_capture_ = std::thread(&Webcam::captureFrame, this);
	}

	Webcam::~Webcam() { }

	void Webcam::printInfo()
	{
		/*auto sensors = dev_.query_sensors();
		for (rs2::sensor& sensor : sensors) {
			std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
			for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
				if (profile.is<rs2::video_stream_profile>()) {
					rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
					std::cout << " stream " << video_stream_profile.stream_name() << " " << video_stream_profile.format() << " " <<
						video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << " Hz\n";
				}
				if (profile.is<rs2::motion_stream_profile>()) {
					rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
					std::cout << " stream " << motion_stream_profile.stream_name() << " " << motion_stream_profile.format() << " " <<
						motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << " Hz\n";
				}
				if (profile.is<rs2::pose_stream_profile>()) {
					rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
					std::cout << " stream " << pose_stream_profile.stream_name() << " " << pose_stream_profile.format() << " " <<
						pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << " Hz\n";
				}
				//std::cout << "  stream " << profile.stream_type() << " " << profile.stream_name() << " " << profile.format() << " @" << profile.fps() << " Hz" << std::endl;
			}
		}
		std::cout << "\n";
		return;*/
	}

	/*void Webcam::setDisparity(bool disparity)
	{
		/*DISPARITY_ = disparity;
		return;
	}*/

	/*void Webcam::start(bool disparity)
	{
		serialnumber_ = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		std::cout << "Opening pipeline for " << serialnumber_ << "\n";
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
		return;
	}*/

	void Webcam::captureFrame()
	{
		// CV_WRAP explicit VideoCapture(int index, int apiPreference, const std::vector<int>& params);

		VCap cap;
		cap.dev_.open(0);
		if (!cap.dev_.isOpened()) {
			std::cerr << "ERROR: Could not open the webcam.\n";
		}

		cap.dev_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
		//cam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2'));
		cap.dev_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
		cap.dev_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.dev_.set(cv::CAP_PROP_FPS, 30);

		std::cout << "Device found:\n";
		/*std::cout << dev_.get(RS2_CAMERA_INFO_NAME) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
			<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << "\n\n";*/
		std::cout << cap.dev_.get(cv::CAP_PROP_FOURCC) << " "
			<< cap.dev_.get(cv::CAP_PROP_FRAME_WIDTH) << " "
			<< cap.dev_.get(cv::CAP_PROP_FRAME_HEIGHT) << " "
			<< cap.dev_.get(cv::CAP_PROP_FPS) << "\n\n";

		std::cout << "A new Webcam was created.\n" << std::endl;

		//this->start(DISPARITY_);
		
		while (true) {
			std::time_t t0, tf;
			t0 = std::time(0);
			
			// Copy the next frame from the camera
			//std::shared_ptr<cv::Mat> image_ptr(new cv::Mat());
			cv::Mat image;
			std::cout << "One\n" << cap.dev_.get(cv::CAP_PROP_FPS) << '\n';
			cap.dev_.read(image);
			std::cout << "Two\n";
			
			// Assign the image frame to the camera
			mu_image_.lock();
			*image_ = image;
			mu_image_.unlock();

			std::cout << "Three\n";
			// Feed the children Sensors
			this->feedChildren();

			tf = std::time(0);
		}
		return;
	}

	/*std::tuple<uint8_t, uint8_t, uint8_t> Webcam::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
	{
		const int w = texture.get_width(), h = texture.get_height();
		int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
		int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);
		int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
		const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
		return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
	}*/

	/*void Webcam::points_to_pcl(const rs2::points pts, const rs2::video_frame color)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr(new pcl::PointCloud<pcl::PointXYZRGB>());

		// rs2::points
		auto sp = pts.get_profile().as<rs2::video_stream_profile>();
		auto ptr_v = pts.get_vertices();
		auto ptr_tc = pts.get_texture_coordinates();

		// pcl::PointXYZ
		cloud_raw->width = sp.width();
		cloud_raw->height = sp.height();
		cloud_raw->is_dense = false;
		cloud_raw->points.resize(pts.size());

		// pcl::PointXYZRGB
		cloud_clr->width = sp.width();
		cloud_clr->height = sp.height();
		cloud_clr->is_dense = false;
		cloud_clr->points.resize(pts.size());

		for (size_t i(0); i < pts.size(); ++i) {
			// pcl::PointXYZ
			cloud_raw->points[i].x = ptr_v[i].x;
			cloud_raw->points[i].y = ptr_v[i].y;
			cloud_raw->points[i].z = ptr_v[i].z;

			// pcl::PointXYZRGB
			cloud_clr->points[i].x = ptr_v[i].x;
			cloud_clr->points[i].y = ptr_v[i].y;
			cloud_clr->points[i].z = ptr_v[i].z;
			// Intel Realsense RGB_Texture conversion available at
			// https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp
			std::tuple<uint8_t, uint8_t, uint8_t> rgb(get_texcolor(color, ptr_tc[i]));
			cloud_clr->points[i].r = std::get<2>(rgb);
			cloud_clr->points[i].g = std::get<1>(rgb);
			cloud_clr->points[i].b = std::get<0>(rgb);
		}

		mu_cloud_.lock();
		cloud_ = cloud_raw;
		cloud_clr_ = cloud_clr;
		mu_cloud_.unlock();
		return;
	}*/
}