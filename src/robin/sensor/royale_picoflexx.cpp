#include "royale_picoflexx.h"

namespace robin {

	RoyalePicoflexx::RoyalePicoflexx()
	{
		data_ = new royale::DepthData();
		
		thread_exposure_ = std::thread(&RoyalePicoflexx::exposure, this);
		//thread_capture_ = std::thread(&RoyalePicoflexx::captureFrame, this);
	}

	RoyalePicoflexx::~RoyalePicoflexx()
	{
		// Stop capture mode
		if (dev_->stopCapture() != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Error stopping the capturing" << std::endl;
			return;
		}
	}

	void RoyalePicoflexx::printInfo()
	{
		/* Yet to be implemented. */
	}

	void RoyalePicoflexx::setDisparity(bool disparity)
	{
		/* Not applicable to RoyalPicoflexx camera. */
		DISPARITY_ = disparity;
	}

	void RoyalePicoflexx::start(bool disparity)
	{
		// Start the capture mode
		if (dev_->startCapture() != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Error starting the capturing" << std::endl;
			return;
		}
	}

	void RoyalePicoflexx::captureFrame()
	{
		while (true) {
			std::cout << "RoyalePicoflexx.captureFrame()" << std::endl;

			std::time_t t0, tf;
			t0 = std::time(0);

			// Wait for the next set of frames from the camera
			mu_data_.lock();
			royale::DepthData* data = new royale::DepthData(*data_);
			mu_data_.unlock();

			// Generate the pointcloud and texture mappings
			//

			// Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
			this->points_to_pcl(data, 128);

			// Feed the children Sensors
			this->feedChildren();

			tf = std::time(0);
			//std::cout << "Read pointcloud from " << cloud_->size() << " data points (in " << std::difftime(t0, tf) / 1000 << " ms)." << std::endl;
		}
	}

	void RoyalePicoflexx::exposure()
	{
		// Windows requires that the application allocate these, not the DLL.  We expect typical
		// Royale applications to be using a GUI toolkit such as Qt, which has its own equivalent of this
		// PlatformResources class (automatically set up by the toolkit).
		sample_utils::PlatformResources resources;

		// This is the data listener which will receive callbacks.  It's declared
		// before the cameraDevice (dev_) so that, if this function exits with a 'return'
		// statement while the camera is still capturing, it will still be in scope
		// until the cameraDevice (dev_)'s destructor implicitly deregisters the listener.
		std::unique_ptr<Listener> listener;

		std::cout << "Waiting for device/s..." << std::endl;
		// The camera manager queries for a connected camera
		royale::CameraManager manager;
		auto camlist = manager.getConnectedCameraList();
		std::cout << "Detected " << camlist.size() << " camera(s)." << std::endl;
		if (!camlist.empty()) {
			std::cout << "CamID for first device: " << camlist.at(0).c_str() << " with a length of (" << camlist.at(0).length() << ")" << std::endl;
			dev_ = manager.createCamera(camlist[0]);
		}

		// Checks if the camera object has been allocated
		if (dev_ != nullptr) {
			std::cout << "A new RoyalePicoflexx was created\n" << std::endl;
		}
		else {
			std::cerr << "Cannot create the camera device" << std::endl;
			return;
		}

		// IMPORTANT: call the initialize method before working with the camera device
		if (dev_->initialize() != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Cannot initialize the camera device" << std::endl;
			return;
		}

		royale::Vector<royale::String> useCases;
		auto status = dev_->getUseCases(useCases);

		if (status != royale::CameraStatus::SUCCESS || useCases.empty())
		{
			std::cerr << "No use cases are available" << std::endl;
			std::cerr << "getUseCases() returned: " << getErrorString(status) << std::endl;
			return;
		}

		// Set an operation mode
		auto selectedUseCaseIdx = 0; // Default operation mode
		if (dev_->setUseCase(useCases.at(selectedUseCaseIdx)) != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Error setting use case" << std::endl;
			return;
		}

		// Retrieve the IDs of the different streams
		if (dev_->getStreams(streamIds_) != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Error retrieving streams" << std::endl;
			return;
		}

		std::cout << "Device found with Stream IDs: ";
		for (auto curStream : streamIds_)
		{
			std::cout << curStream << ", ";
		}
		std::cout << std::endl;

		// Register a data listener
		listener.reset(new Listener(streamIds_, *this));
		if (dev_->registerDataListener(listener.get()) != royale::CameraStatus::SUCCESS)
		{
			std::cerr << "Error registering data listener" << std::endl;
			return;
		}

		this->start(DISPARITY_);

		// Let the camera capture for some time
		std::this_thread::sleep_for(std::chrono::seconds(5));

		// Change the exposure time for the first stream of the use case (Royale will limit this to an
		// eye-safe exposure time, with limits defined by the use case).  The time is given in
		// microseconds.
		//
		// Non-mixed mode use cases have exactly one stream, mixed mode use cases have more than one.
		// For this example we only change the first stream.
		if (dev_->setExposureTime(300, streamIds_[0]) != royale::CameraStatus::SUCCESS) { //200
			std::cerr << "Cannot set exposure time for stream" << streamIds_[0] << std::endl;
		}
		else {
			std::cout << "Changed exposure time for stream " << streamIds_[0] << " to 200 microseconds ..." << std::endl;
		}

		// Let the camera capture for some time (an hour)
		std::this_thread::sleep_for(std::chrono::seconds(3600));
	}

	void RoyalePicoflexx::points_to_pcl(const royale::DepthData* data, uint8_t depthConfidence)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		cloud->width = data->width;
		cloud->height = data->height;
		cloud->is_dense = false;
		cloud->points.resize(data->points.size());
		auto ptr = data->points.begin();
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