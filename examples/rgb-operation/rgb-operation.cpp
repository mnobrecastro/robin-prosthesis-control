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

int main(int argc, char** argv)
{	
	// Declare a solver3
	robin::Solver3 mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315);
	//mysolver.setDownsample(0.002f);
	mysolver.setPlaneRemoval(false);

	// Create a sensor from a camera
	//robin::CameraRgb* mycam(new robin::CameraRgb);
	const std::string url("http://192.168.87.188:5000/video_feed"); // Home
	//const std::string url("http://172.26.24.220:5000/video_feed"); // AAU
	robin::Webcam* webcam(new robin::Webcam(url));
	//mycam->printInfo();
	//mycam->setDisparity(false);

	// Declare and instanciate a segmentation object
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_RANSAC);
	seg->setMaxIterations(1000);
	seg->setDistanceThreshold(0.005); // little slack
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

	std::string win_depth("Depthmap");
	cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
	std::string win_mask("Mask");
	cv::namedWindow(win_mask, cv::WINDOW_AUTOSIZE);

	////

	robin::Sensor3* depthcam(new robin::Sensor3);
	mysolver.addSensor(depthcam);


	// To avoid a "cv::Exception at memory location" before the camera is initialized in a secondary thread
	// a couple of waiting seconds should be considered before entering the main thread loop below.
	cv::waitKey(5000);

	std::vector<double> freq;
	while (true) {
		auto tic = std::chrono::high_resolution_clock::now();

		/*mysolver.solve(prim);

		viewer->removeAllShapes();
		viewer->removeAllPointClouds();

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> preproc_color_h(255, 255, 255);
		preproc_color_h.setInputCloud(mysolver.getPreprocessed());
		viewer->addPointCloud(mysolver.getPreprocessed(), preproc_color_h, "preproc");

		///

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> solver_color_h(0, 255, 0);
		solver_color_h.setInputCloud(mysolver.getPointCloud());
		viewer->addPointCloud(mysolver.getPointCloud(), solver_color_h, "solver");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
		primitive_color_h.setInputCloud(prim->getPointCloud());
		viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
		prim->visualize(viewer);

		viewer->spinOnce(1, true);
		*/

		cv::imshow(wname, *(webcam->getImage()));

		// Rescale the ouput image
		//cv::resize(img_out, img_out, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 0, 0, cv::INTER_AREA);

		cv::Mat im(*(webcam->getImage())); // CV_8UC3
		int H(im.size().height), W(im.size().width);
		// Cropping the 'depthmap' image
		cv::Mat im_depth = im(cv::Range(H/4, 3*H/4), cv::Range(0, W/2));
		// Cropping the 'mask' image
		cv::Mat im_mask = im(cv::Range(H/4, 3*H/4), cv::Range(W/2, W));

		// Render both images
		cv::imshow(win_depth, im_depth);
		cv::imshow(win_mask, im_mask);

		cv::waitKey(1);

		// 

		// RS Camera Intrinsics
		float rx(424), ry(240); // Resolution
		float cx(211.937), cy(122.685); // Center dist
		float f(211.357); // Focal lenght
		float scaling(1000.0); // Scaling factor

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask(new pcl::PointCloud<pcl::PointXYZ>());
		for (size_t y(0); y < im_depth.size().height; ++y) {
			for (size_t x(0); x < im_depth.size().width; ++x) {
				// Recall BGR ordering				
				float pZ = (im_depth.at<cv::Vec3b>(y, x).val[0] + im_depth.at<cv::Vec3b>(y, x).val[1] + im_depth.at<cv::Vec3b>(y, x).val[2]) / scaling;
				float pX = (x - cx) * pZ / f;
				float pY = (y - cy) * pZ / f;
				if (pZ > 0.100) {
					cloud_raw->push_back(pcl::PointXYZ(pX, pY, pZ));
				}
			}
		}
		std::cout << "cloud: " << cloud_raw->size() << "\n";


		cv::Mat affordance_ch;
		cv::extractChannel(im_mask, affordance_ch, 0); // Graspable-only
		cv::Mat affordance_mask;
		cv::threshold(affordance_ch, affordance_mask, 1, 255, cv::THRESH_BINARY);

		cv::Mat im_blobs, stats, centroids; // Graspable-only
		int n_blobs = cv::connectedComponentsWithStats(affordance_mask, im_blobs, stats, centroids, 8);
		double min_dist(std::numeric_limits<float>::max());
		int blob_idx(0); // index of the center-most blob
		int min_area(0.01 * rx * ry); // 10% of the image area in px
		for (int k(1); k < n_blobs; ++k) {
			double dist = std::sqrt((centroids.at<double>(k,0) - cx) * (centroids.at<double>(k, 0) - cx) + (centroids.at<double>(k,1) - cy) * (centroids.at<double>(k, 1) - cy));
			if (dist < min_dist && stats.at<int>(k, cv::CC_STAT_AREA) > min_area) {
				blob_idx = k;
				min_dist = dist;
			}
		}

		// Choose the center-most blob among those found by the connectedComponents
		cv::Mat affordance_blob = cv::Mat::zeros(im_blobs.size().height, im_blobs.size().width, CV_8UC1);
		if (blob_idx > 0) {
			for (size_t y(0); y < im_blobs.size().height; ++y) {
				for (size_t x(0); x < im_blobs.size().width; ++x) {
					if (im_blobs.at<int>(y,x) == blob_idx) {
						affordance_blob.at<uchar>(y, x) = 255;
					}
				}
			}
		}

		// Reconstruct the point cloud by masking the depthdata with the affordance blob
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask(new pcl::PointCloud<pcl::PointXYZ>());
		cv::Mat affordance_depth;
		im_depth.copyTo(affordance_depth, affordance_blob);
		for (size_t x(0); x < affordance_depth.size().width; ++x) {
			for (size_t y(0); y < affordance_depth.size().height; ++y) {
				// Recall BGR ordering				
				float pZ = (affordance_depth.at<cv::Vec3b>(y, x).val[0] + affordance_depth.at<cv::Vec3b>(y, x).val[1] + affordance_depth.at<cv::Vec3b>(y, x).val[2]) / scaling;
				float pX = (x - cx) * pZ / f;
				float pY = (y - cy) * pZ / f;
				if (pZ > 0.100) {
					cloud_mask->push_back(pcl::PointXYZ(pX, pY, pZ));
				}
			}
		}
		std::cout << "mask: " << cloud_mask->size() << "\n";


		// ----------------------------------------------------
		// Process the affordance point cloud

		depthcam->setPointCloud(*cloud_mask);

		// Reset the dummy Primitive3d3 for multiple primitive inference
		prim = new robin::Primitive3d3;

		mysolver.solve(prim);

		viewer->removeAllShapes();
		viewer->removeAllPointClouds();

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(0, 0, 255);
		cloud_color_h.setInputCloud(cloud_raw);
		viewer->addPointCloud(cloud_raw, cloud_color_h, "cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mask_color_h(0, 255, 0);
		mask_color_h.setInputCloud(cloud_mask);
		viewer->addPointCloud(cloud_mask, mask_color_h, "mask");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
		primitive_color_h.setInputCloud(prim->getPointCloud());
		viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
		prim->visualize(viewer);

		viewer->spinOnce(1, true);


		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0 / t.count() << " Hz (in " << t.count() * 1000.0 << " ms).\n";
		freq.push_back(t.count());
	}

	// Destroy all OpenCV windows
	cv::destroyWindow(wname);
	cv::destroyWindow(win_depth);
	cv::destroyWindow(win_mask);

	return 0;
}