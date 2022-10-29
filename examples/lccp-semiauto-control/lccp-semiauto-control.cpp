/*
 * Semi-autonomous Prosthesis Control Using Computer Vision
 * (Robin C++ framework)
 *
 * "lccp-multiprimitive.cpp"
 *
 * The provided example shows how to track an object using the LCCP algorithm
 * followed by a RANSAC fitting of a geometric primitive model (simultaneously
 * inferring the best fit between a spherical, a cylindrical or a cuboid model).
 *
 * Author: Miguel Nobre Castro (mnobrecastro@gmail.com)
 *
 *
 * This work was performed at the Department of Health Science and Technology,
 * Aalborg University, under the supervision of Professor Strahinja Dosen
 * (sdosen@hst.aau.dk). The Independent Research Fund Denmark supported it through
 * the project ROBIN "RObust Bidirectional human-machine INterface for natural
 * control and feedback in hand prostheses" (8022-00243A).
 */

//#define MULTITHREADING
//#define GNUPLOT

#include <robin/utils/data_manager.h>
#include <robin/sensor/hand_michelangelo.h>
#include <robin/control/control_continuous.h>
#include <robin/solver/solver3.h>
#include <robin/solver/solver3_lccp.h>
#include <robin/solver/solver3_lasers.h>
#include <robin/sensor/realsense_d400.h>
#include <robin/sensor/royale_picoflexx.h>
#include <robin/sensor/laser_scanner.h>
#include <robin/sensor/laser_array.h>
#include <robin/primitive/primitive3_sphere.h>
#include <robin/primitive/primitive3_cylinder.h>
#include <robin/primitive/primitive3_cuboid.h>
#include <robin/primitive/primitive3_line.h>
#include <robin/primitive/primitive3_circle.h>

#include <chrono>
#include <thread>

#ifdef GNUPLOT
#include "gnuplot-iostream/gnuplot-iostream.h"
#endif

#pragma comment(lib,"robin.lib")

int main(int argc, char** argv)
{
	// ---
	robin::data::DataManager mydm;
	// ---
	
	Beep(1000, 100); Beep(0, 100); Beep(2000, 100); Beep(0, 100); Beep(3000, 100); Beep(0, 100); Beep(4000, 100);

	robin::hand::Michelangelo myhand(false);
	myhand.setDataManager(mydm);
	myhand.plotEMG(false);
	myhand.calibrateEMG();

	robin::control::ControlContinuous controller(myhand);
	controller.setFilter(robin::control::ControlVar::fname::MOVING_AVERAGE, 10); //20=~200ms     //MEDIAN, 40
	controller.setFullManual(false);
	controller.setDataManager(mydm);

	// Declare a solver3
	robin::Solver3LCCP mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315); //0.105 or 0.160 //(0.115, 0.215)
	mysolver.setDownsample(0.002f);
	//mysolver.setResample(2, 0.005);
	mysolver.setPlaneRemoval(false);
	mysolver.setFairSelection(false);
	//solver.setUseNormals(true);
	
	// Dummy Segmentation object
	//pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>* seg(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);
	//seg->setNormalDistanceWeight(0.001);
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_PROSAC); //PROSAC?
	seg->setMaxIterations(1000); //100
	seg->setDistanceThreshold(0.001); //0.001 //0.0005
	seg->setRadiusLimits(0.005, 0.050);
	mysolver.setSegmentation(seg);
	
	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	//robin::RoyalePicoflexx* mycam(new robin::RoyalePicoflexx());
	mycam->printInfo();
	mycam->setDisparity(false);
	mysolver.addSensor(mycam);

	// Create a Primitive
	robin::Primitive3d3* prim;
	//robin::Primitive3d3* prim(new robin::Primitive3Cylinder);	
	//prim->setVisualizeOnOff(false);

	// Create a PCL visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	viewer->setBackgroundColor(0.91, 0.96, 0.97, vp);
	viewer->addCoordinateSystem(0.1);

#ifdef GNUPLOT
	// Create a Gnuplot canvas
	Gnuplot gp;
	std::vector<std::pair<double, double>> gnup_grasp_size, gnup_tilt_angle, gnup_emg1, gnup_emg2;
	size_t kdata(0);
#endif

	bool RENDER(true);
	bool PLOT(false);
	bool HAND_CONTROL(true);
	std::vector<double> freq;

	while(true){
		auto tic = std::chrono::high_resolution_clock::now();

		// Reset the dummy Primitive3d3 for multiple primitive inference
		prim = new robin::Primitive3d3;
		
		mysolver.solve(prim);

		///
		if (HAND_CONTROL) {
			controller.evaluate(prim);
			std::cout << "Target grasp_size: " << controller.getGraspSize() << std::endl;
			std::cout << "Target tilt_angle: " << controller.getTiltAngle() << " (" << controller.getTiltAngle() * 180.0 / 3.14159 << ")" << std::endl;
#ifdef GNUPLOT
			if (PLOT) {
				gnup_grasp_size.emplace_back(kdata, controller.getGraspSize());
				gnup_tilt_angle.emplace_back(kdata, controller.getTiltAngle() * 180.0 / 3.14159);
			}
#endif

			if (myhand.isRightHand()) {
				// Right-hand prosthesis (positive tilt angle)
				std::cout << "Hand grasp_size: " << myhand.getGraspSize() << std::endl;
				std::cout << "Hand tilt_angle: " << myhand.getWristSupProAngle() << " (" << myhand.getWristSupProAngle() * 180.0 / 3.14159 << ")" << std::endl;
			}
			else {
				// Left-hand prosthesis (negative tilt angle)
				std::cout << "Hand grasp_size: " << myhand.getGraspSize() << std::endl;
				std::cout << "Hand tilt_angle: " << -myhand.getWristSupProAngle() << " (" << -myhand.getWristSupProAngle() * 180.0 / 3.14159 << ")" << std::endl;
			}
			if (PLOT) {
				//gnup_emg1.emplace_back(kdata, controller.getEMG()[0]);
				//gnup_emg2.emplace_back(kdata, controller.getEMG()[1]);
				//gnup_emg1.emplace_back(kdata, *(myhand.getEMGSolvers()[0]->getPreprocessed().end()));
				//gnup_emg2.emplace_back(kdata, *(myhand.getEMGSolvers()[1]->getPreprocessed().end()));
			}

			std::cout << "Bools: ";
			if (controller.getStateAuto()) {
				std::cout << "ON";
			}
			else {
				std::cout << "OFF";
			}
			std::cout << " ";
			if (controller.getStateGrasp()) {
				std::cout << "ON";
			}
			else {
				std::cout << "OFF";
			}
			std::cout << std::endl;

			std::cout << "\n" << std::endl;
		}
		///

		//---- RENDERING ----
		if (RENDER) {
			// Clean any previously rendered objects
			viewer->removeAllShapes();
			viewer->removeAllPointClouds();

			/*for (auto s : mysolver.getSensors()) {
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(255, 255, 255);
				cloud_color_h.setInputCloud(s->getPointCloud());
				viewer->addPointCloud(s->getPointCloud(), cloud_color_h, std::to_string(std::rand()));
			}*/
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> preproc_color_h(0, 0, 0);
			preproc_color_h.setInputCloud(mysolver.getPreprocessed());
			viewer->addPointCloud(mysolver.getPreprocessed(), preproc_color_h, "preproc");

			///

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> solver_color_h(0, 255, 0);
			solver_color_h.setInputCloud(mysolver.getPointCloud());
			viewer->addPointCloud(mysolver.getPointCloud(), solver_color_h, "solver");
			mysolver.visualize(viewer); //lccp:"marker"

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
			primitive_color_h.setInputCloud(prim->getPointCloud());
			viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
			prim->visualize(viewer);

			//std::cout << *prim->getCoefficients() << std::endl;

			//------
			/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(255, 255, 255);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_color(mycam->getPointCloud());
			cloud_color_h.setInputCloud(cloud_color);
			viewer->addPointCloud(cloud_color, cloud_color_h);*/
			//-----

			viewer->spinOnce(1, true);
		}

#ifdef GNUPLOT
		//---- PLOTTING ----
		if (PLOT) {

			// Gnuplots
			//gp << "set multiplot layout 2, 1 rowsfirst";

			/*gp << "set yrange [0.0:0.5]\n";
			//gp << "unset key";
			gp << "plot '-' with lines title 'gnup_grasp_size'\n";
			gp.send1d(gnup_grasp_size);*/
			
			//gp << "set yrange [-180.0:180.0]\n";
			////gp << "unset key";
			//gp << "plot '-' with lines title 'gnup_tilt_angle'\n";
			//gp.send1d(gnup_tilt_angle);

			// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
			gp << "set yrange [0.0:1.0]\n";
			gp << "plot '-' with lines title 'emg1', '-' with lines title 'emg2'\n";
			gp.send1d(gnup_emg1);
			gp.send1d(gnup_emg2);
			
			//gp << "unset multiplot";
			gp.flush();
			++kdata;
		}
#endif

		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0/t.count() << " Hz (in " << t.count()*1000.0 << " ms).\n" << std::endl;
		freq.push_back(1.0/t.count());
	}	
}