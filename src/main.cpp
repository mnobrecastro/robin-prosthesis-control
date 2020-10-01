#define MULTITHREADING

#include "../src/hand_michelangelo.h"
#include "../src/control_simple.h"
#include "../src/solver3.h"
#include "../src/solver3_lccp.h"
#include "../src/solver3_lasers.h"
#include "../src/realsense_d400.h"
#include "../src/royale_picoflexx.h"
#include "../src/laser_scanner.h"
#include "../src/laser_array.h"
#include "../src/primitive3_sphere.h"
#include "../src/primitive3_cylinder.h"
#include "../src/primitive3_cuboid.h"
#include "../src/primitive3_line.h"
#include "../src/primitive3_circle.h"

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	//std::cout << '\a';
	Beep(523, 100); // 523 hertz (C5) for 500 milliseconds
	//ascending pitch beep
	/*for (int i = 0; i < 3000; i += 10) {
		Beep(i, 100);
	}*/
	//descending pitch beep
	/*for (int i = 3000; i > 0; i -= 10) {
		Beep(i, 100);
	}*/
	
	// Create a hand
	//robin::hand::HandUDP myhand(false, "127.0.0.1", 8052, 8051);
	//system("C:\\Users\\MMC\\Documents\\AAU\\Projects\\Robin\\Software\\Mikey\\DLLs\\MichelangeloGUI.exe");

	//uint8_t packet[9] = { 1, 20,0,0,0, 20,0,0,0 };
	//uint8_t packet[1] = { 0 };
	//myhand.send_packet(packet, sizeof(packet)/sizeof(uint8_t));		
	
	//uint8_t packpack[1024];
	//int byte_len = myhand.receive_packet(packpack);
	//myhand.print_recv_packet(packpack, byte_len);


	robin::hand::Michelangelo myhand(false);

	//robin::hand::Hand myhand(TRUE);

	robin::control::ControlSimple controller(myhand);
	controller.setFilter(robin::control::ControlVar::fname::MEDIAN, 40);
	// Declare a solver3
	robin::Solver3LCCP mysolver;
	//robin::Solver3Lasers mysolver;
	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.215); //0.105 or 0.160
	mysolver.setDownsample(0.002f); //dflt=0.005f //Cyl=0.0025f //Cub=0.005f   //0.004f
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
	seg->setDistanceThreshold(0.0005); //0.001
	seg->setRadiusLimits(0.005, 0.050);
	mysolver.setSegmentation(seg);
	
	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	//robin::RoyalePicoflexx* mycam(new robin::RoyalePicoflexx());
	mycam->printInfo();
	mycam->setDisparity(false);
	mysolver.addSensor(mycam);

	//robin::LaserArrayCross* myarr(new robin::LaserArrayCross(mycam, 0.001));
	//mysolver.addSensor(myarr);

	//-----
	// Create a sensor from another sensor
	/*robin::LaserScanner* mylaser_h(new robin::LaserScanner(mycam, 0.0, 1.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(mylaser_h);
	robin::LaserScanner* mylaser_v(new robin::LaserScanner(mycam, 1.0, 0.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(mylaser_v);*/

	/*robin::LaserScanner* laser_0(new robin::LaserScanner(mycam, 0.0, 1.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(laser_0);
	robin::LaserScanner* laser_1(new robin::LaserScanner(mycam, -1.0, 1.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(laser_1);
	robin::LaserScanner* laser_2(new robin::LaserScanner(mycam, 1.0, 0.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(laser_2);
	robin::LaserScanner* laser_3(new robin::LaserScanner(mycam, 1.0, 1.0, 0.0, 0.0, 0.001));
	mysolver.addSensor(laser_3);*/
	//-----

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
	float bckgr_gray_level = 0.0;  // Black:=0.0
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	viewer->addCoordinateSystem(0.25);

	bool RENDER(true);
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
			std::cout << "Grasp_size: " << controller.getGraspSize() << std::endl;
			std::cout << "Tilt_angle: " << controller.getTiltAngle() << " (" << controller.getTiltAngle() * 180.0 / 3.14159 << ")" << std::endl;

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
			//mysolver.visualize(viewer); //lccp:"marker"

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

		//---- PROFILING ---
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
		std::cout << "Cycle duration: " << 1.0/t.count() << " Hz (in " << t.count()*1000.0 << " ms).\n" << std::endl;
		freq.push_back(t.count());
	}	
}