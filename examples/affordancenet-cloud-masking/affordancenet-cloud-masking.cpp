/* ROBIN SEMI-AUTONOMOUS PROSTHESIS CONTROL LIBRARY
 * The example provided shows how to track an object by approximating it
 * to a geometric primitive of a cylindrical object.
 */


//#include <robin/solver/solver2.h>
#include <robin/sensor/camera_rgb.h>
#include <robin/sensor/webcam.h>
//#include "robin/solver/solver3_lasers.h"
//#include <robin/sensor/laser_array.h>
//#include <robin/primitive/primitive3_cylinder.h>

#include <robin/sensor/sensor3.h>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <limits>

#include <pcl/visualization/pcl_visualizer.h>
#include <robin/solver/solver3.h>
#include <robin/solver/solver3_affnet.h>
#include <robin/solver/solver3_lccp.h>


int main(int argc, char** argv)
{	
	// Declare a solver3
	//robin::Solver3 mysolver;
	robin::Solver3LCCP mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.215); //0.215
	//mysolver.setCrop(-0.25, 0.25, -0.25, 0.25, 0.115, 0.5); //0.215
	mysolver.setDownsample(0.002f);
	mysolver.setDenoise(50, 0.1); //50, 0.1
	////mysolver.setResample(2, 0.1f);
	//mysolver.setPlaneRemoval(false);

	// Create a sensor from a camera
	//robin::CameraRgb* mycam(new robin::CameraRgb);
	const std::string url("http://192.168.87.107:5000/video_feed"); // Home
	//const std::string url("http://192.168.1.33:5000/video_feed"); // Home2
	//const std::string url("http://172.26.24.202:5000/video_feed"); // AAU-Office
	//const std::string url("http://172.25.151.90:5000/video_feed"); // AAU-Lab_A2_109
	robin::Webcam* webcam(new robin::Webcam(url));
	mysolver.addSensor(webcam);
	//mycam->printInfo();
	//mycam->setDisparity(false);
	
	// RS Camera Intrinsics
	struct CamIntrinsics {
		int rx, ry; // Resolution
		float cx, cy; // Center dist
		float f; // Focal lenght
		float s; // Scaling factor
	};

	const CamIntrinsics Cam = { 424, 240, 211.937, 122.685, 211.357, 1000.0 };
	//Cam.rx = 424;
	//Cam.ry = 240;
	//Cam.cx = 211.937;
	//Cam.cy = 122.685;
	//Cam.f = 211.357;
	//Cam.s = 1000.0;

	// Declare and instanciate a segmentation object
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_PROSAC);
	seg->setMaxIterations(1000);
	seg->setDistanceThreshold(0.001); // 0.001
	seg->setRadiusLimits(0.005, 0.050);
	mysolver.setSegmentation(seg);

	// Create a Primitive
	robin::Primitive3d3* prim;


	// Create a PCL visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	viewer->setBackgroundColor(0.91, 0.96, 0.97, vp);
	viewer->addCoordinateSystem(0.1);


	// Create an OpenCV rendering window
	std::string wname("Input");
	cv::namedWindow(wname, cv::WINDOW_AUTOSIZE);

	////

	/*robin::Sensor3* depthcam(new robin::Sensor3);
	mysolver.addSensor(depthcam);*/


	// To avoid a "cv::Exception at memory location" before the camera is initialized in a secondary thread
	// a couple of waiting seconds should be considered before entering the main thread loop below.
	cv::waitKey(5000);

	bool RENDER(true);
	std::vector<double> freq;

	while (true) {
		auto tic = std::chrono::high_resolution_clock::now();

		// Reset the dummy Primitive3d3 for multiple primitive inference
		prim = new robin::Primitive3d3;

		mysolver.solve(prim);
		//mysolver.principal_components(prim);

		if (prim->isEmpty()) {
			std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";
		}

		//---- RENDERING ----
		if (RENDER) {

			cv::imshow(wname, *(webcam->getImage()));
			cv::waitKey(1);

			viewer->removeAllShapes();
			viewer->removeAllPointClouds();

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(0, 0, 255);
			cloud_color_h.setInputCloud(mysolver.getPreprocessed());
			viewer->addPointCloud(mysolver.getPreprocessed(), cloud_color_h, "cloud");

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mask_color_h(0, 255, 0);
			mask_color_h.setInputCloud(mysolver.getPointCloud());
			viewer->addPointCloud(mysolver.getPointCloud(), mask_color_h, "mask");

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
			primitive_color_h.setInputCloud(prim->getPointCloud());
			viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
			prim->visualize(viewer);

			viewer->spinOnce(1, true);
		}

		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0 / t.count() << " Hz (in " << t.count() * 1000.0 << " ms).\n";
		freq.push_back(t.count());
	}

	// Destroy all OpenCV windows
	cv::destroyWindow(wname);

	return 0;
}