//#define MULTITHREADING
//#define GNUPLOT

#include <robin/utils/data_manager.h>
#include <robin/sensor/hand_michelangelo.h>
#include <robin/control/control_simple.h>
#include <robin/solver/solver3.h>
#include <robin/solver/solver3_lccp.h>
#include <robin/solver/solver3_lasers.h>
#include <robin/sensor/realsense_d400.h>
//#include <robin/sensor/royale_picoflexx.h>
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

int main(int argc, char** argv)
{
	// ---
	robin::data::DataManager mydm;
	// ---
	
	Beep(523, 100); 

	robin::hand::Michelangelo myhand(true);
	myhand.setDataManager(mydm);
	myhand.plotEMG(false);
	myhand.calibrateEMG();

	robin::control::ControlSimple controller(myhand);
	controller.setFilter(robin::control::ControlVar::fname::MOVING_AVERAGE, 20); //20=~200ms     //MEDIAN, 40
	controller.setFullManual(false);
	controller.setDataManager(mydm);

	// Declare a solver3
	robin::Solver3Lasers mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315);
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
	seg->setMethodType(pcl::SAC_PROSAC);
	seg->setMaxIterations(1000);
	seg->setDistanceThreshold(0.001);
	seg->setRadiusLimits(0.005, 0.050);
	mysolver.setSegmentation(seg);
	
	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	//robin::RoyalePicoflexx* mycam(new robin::RoyalePicoflexx());
	mycam->printInfo();
	mycam->setDisparity(false);

	// Create a virtual array of sensors from another sensor
	robin::LaserArrayStar* myarr(new robin::LaserArrayStar(mycam, 0.001));
	mysolver.addSensor(myarr);

	// Create a Primitive
	//robin::Primitive3d3* prim;
	robin::Primitive3d3* prim(new robin::Primitive3Cylinder);	
	//prim->setVisualizeOnOff(false);

	// Create a PCL visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	float bckgr_gray_level = 0.0;  // Black:=0.0
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
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
		//prim = new robin::Primitive3d3;
		
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
		freq.push_back(t.count());
	}	
}