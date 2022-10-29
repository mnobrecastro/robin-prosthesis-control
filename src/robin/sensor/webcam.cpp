/*
 * Semi-autonomous Prosthesis Control Using Computer Vision - Robin C++ framework
 *
 * Author: Miguel Nobre Castro (mnobrecastro@gmail.com)
 *
 *
 * This work was performed at the Department of Health Science and Technology, Aalborg
 * University, under the supervision of Professor Strahinja Dosen (sdosen@hst.aau.dk),
 * and was supported by the Independent Research Fund Denmark through the project ROBIN
 * "RObust Bidirectional human-machine INterface for natural control and feedback in
 * hand prostheses" (8022-00243A).
 */

#include "webcam.h"

namespace robin {

	Webcam::Webcam(int dev_index)
		: dev_(new cv::VideoCapture)
	{
		// Open and check if the camera was successfully initialized
		dev_->open(dev_index);
		if (!dev_->isOpened()) {
			std::cerr << "ERROR: Could not open the webcam.\n";
		}
		std::cout << "Device found:\n";
		std::cout << dev_->get(cv::CAP_PROP_FOURCC) << " "
			<< dev_->get(cv::CAP_PROP_FRAME_WIDTH) << " "
			<< dev_->get(cv::CAP_PROP_FRAME_HEIGHT) << " "
			<< dev_->get(cv::CAP_PROP_FPS) << "\n\n";

		std::cout << "A new Webcam was created.\n" << std::endl;

		thread_capture_ = std::thread(&Webcam::captureFrame, this);
	}

	Webcam::Webcam(const std::string dev_str)
		: dev_(new cv::VideoCapture)
	{
		// Open and check if the camera was successfully initialized
		dev_->open(dev_str);
		if (!dev_->isOpened()) {
			std::cerr << "ERROR: Could not open the webcam.\n";
		}
		std::cout << "Device found:\n";
		std::cout << dev_->get(cv::CAP_PROP_FOURCC) << " "
			<< dev_->get(cv::CAP_PROP_FRAME_WIDTH) << " "
			<< dev_->get(cv::CAP_PROP_FRAME_HEIGHT) << " "
			<< dev_->get(cv::CAP_PROP_FPS) << "\n\n";

		std::cout << "A new Webcam was created.\n" << std::endl;

		thread_capture_ = std::thread(&Webcam::captureFrame, this);
	}

	Webcam::~Webcam() { }

	void Webcam::printInfo() { }

	void Webcam::captureFrame()
	{
		while (true) {
			std::time_t t0, tf;
			t0 = std::time(0);

			// Get the next frame from the camera
			cv::Mat frame;
			dev_->read(frame);
						
			// Assign the image frame to the image_
			mu_image_.lock();
			frame.copyTo(*image_);
			mu_image_.unlock();

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
			cloud_raw->points[i].x = ptr_v[i].x;it 
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