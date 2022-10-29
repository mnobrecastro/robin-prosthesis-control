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

#include <robin/solver/solver3.h>
#include <robin/solver/solver3_lccp.h>
#include <robin/sensor/realsense_d400.h>
//#include <robin/sensor/royale_picoflexx.h> /* UNCOMMENT IF USED */
#include <robin/primitive/primitive3_sphere.h>
#include <robin/primitive/primitive3_cylinder.h>
#include <robin/primitive/primitive3_cuboid.h>
#include <robin/primitive/primitive3_line.h>
#include <robin/primitive/primitive3_circle.h>

#include <chrono>
#include <thread>

#pragma comment(lib,"robin.lib")

int main(int argc, char** argv)
{
	/* "lccp-multiprimitive.cpp"
	 *
	 * The provided example shows how to track an object using the LCCP algorithm
	 * followed by a RANSAC fitting of a geometric primitive model (simultaneously
	 * inferring the best fit between a spherical, a cylindrical or a cuboid model).
	 */

	// Declare a solver3
	robin::Solver3LCCP mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315);
	mysolver.setDownsample(0.002f);
	//mysolver.setResample(2, 0.005);
	mysolver.setPlaneRemoval(false);
	mysolver.setFairSelection(false);
	//solver.setUseNormals(true);
	
	// Dummy Segmentation object
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_PROSAC);
	seg->setMaxIterations(1000); //100
	seg->setDistanceThreshold(0.001);
	seg->setRadiusLimits(0.005, 0.050);
	mysolver.setSegmentation(seg);
	
	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	//robin::RoyalePicoflexx* mycam(new robin::RoyalePicoflexx()); /* UNCOMMENT IF USED */
	mycam->printInfo();
	mycam->setDisparity(false);
	mysolver.addSensor(mycam);

	// Create a Primitive
	robin::Primitive3d3* prim;
	//prim->setVisualizeOnOff(false);


	// Create a PCL visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	viewer->setBackgroundColor(0.91, 0.96, 0.97, vp);
	viewer->addCoordinateSystem(0.1);

	std::vector<double> freq;
	while(true){
		auto tic = std::chrono::high_resolution_clock::now();

		// Reset the dummy Primitive3d3 for multiple primitive inference
		prim = new robin::Primitive3d3;
		
		mysolver.solve(prim);

		//---- RENDERING ----
		// Clean any previously rendered objects
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();

		viewer->addPointCloud(mysolver.getRawColored(), "rawclr");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> preproc_color_h(255, 255, 255);
		preproc_color_h.setInputCloud(mysolver.getPreprocessed());
		//viewer->addPointCloud(mysolver.getPreprocessed(), preproc_color_h, "preproc");

		///

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> solver_color_h(0, 255, 0);
		solver_color_h.setInputCloud(mysolver.getPointCloud());
		//viewer->addPointCloud(mysolver.getPointCloud(), solver_color_h, "solver");
		mysolver.visualize(viewer); //lccp:"marker"

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
		primitive_color_h.setInputCloud(prim->getPointCloud());
		//viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
		prim->visualize(viewer);

		viewer->spinOnce(1, true);

		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0/t.count() << " Hz (in " << t.count()*1000.0 << " ms).\n" << "\n";
		freq.push_back(1.0/t.count());

		std::cin.get();
	}	
}