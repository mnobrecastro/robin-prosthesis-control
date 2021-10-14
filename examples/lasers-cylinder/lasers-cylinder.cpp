/* ROBIN SEMI-AUTONOMOUS PROSTHESIS CONTROL LIBRARY
 * The example provided shows how to track an object by approximating it
 * to a geometric primitive of a cylindrical object.
 */

#include <robin/solver/solver3.h>
#include <robin/sensor/realsense_d400.h>
#include "robin/solver/solver3_lasers.h"
#include <robin/sensor/laser_array.h>
#include <robin/primitive/primitive3_cylinder.h>

int main(int argc, char** argv)
{	
	// Declare a solver3
	robin::Solver3Lasers mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315);
	//mysolver.setDownsample(0.002f);
	mysolver.setPlaneRemoval(false);

	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	mycam->printInfo();
	mycam->setDisparity(false);

	// Create a virtual array of sensors from another sensor
	robin::LaserArraySingle* myarr(new robin::LaserArraySingle(mycam, 0.002)); //0.001
	//robin::LaserArrayCross* myarr(new robin::LaserArrayCross(mycam, 0.002)); //0.001
	mysolver.addSensor(myarr);

	// Segmentation object
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_RANSAC);
	seg->setMaxIterations(1000);
	seg->setDistanceThreshold(0.003); // little slack 0.005
	seg->setRadiusLimits(0.025, 0.500); //0.050
	mysolver.setSegmentation(seg);

	// Create a Primitive
	robin::Primitive3d3* prim(new robin::Primitive3Cylinder());



	// Create a PCL visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	float bckgr_gray_level = 0.0;
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	viewer->addCoordinateSystem(0.1);


	std::vector<double> freq;
	while (true) {
		auto tic = std::chrono::high_resolution_clock::now();

		mysolver.solve(prim);

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

		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0 / t.count() << " Hz (in " << t.count() * 1000.0 << " ms).\n" << std::endl;
		freq.push_back(t.count());
	}
}