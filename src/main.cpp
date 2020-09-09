#include "../src/solver3.h"
#include "../src/solver3_lccp.h"
#include "../src/solver3_lasers.h"
#include "../src/realsense_d400.h"
#include "../src/laser_scanner.h"
#include "../src/laser_array.h"
#include "../src/primitive3_sphere.h"
#include "../src/primitive3_cylinder.h"
#include "../src/primitive3_cuboid.h"
#include "../src/primitive3_line.h"
#include "../src/primitive3_circle.h"
#include "../src/hand_michelangelo.h"
#include "../src/control_simple.h"

#include <chrono>
#include <thread>

#define MULTITHREADING

//int main(int argc, char** argv)
//{
//	// Create a hand
//	//robin::hand::HandUDP myhand(false, "127.0.0.1", 8052, 8051);
//	//system("C:\\Users\\MMC\\Documents\\AAU\\Projects\\Robin\\Software\\Mikey\\DLLs\\MichelangeloGUI.exe");
//
//	//uint8_t packet[9] = { 1, 20,0,0,0, 20,0,0,0 };
//	//uint8_t packet[1] = { 0 };
//	//myhand.send_packet(packet, sizeof(packet)/sizeof(uint8_t));		
//	
//	//uint8_t packpack[1024];
//	//int byte_len = myhand.receive_packet(packpack);
//	//myhand.print_recv_packet(packpack, byte_len);
//
//
//	robin::hand::Michelangelo myhand(false);
//
//	//robin::hand::Hand myhand(TRUE);
//
//	robin::control::ControlSimple controller(myhand);
//
//	// Declare a solver3
//	robin::Solver3LCCP mysolver;
//	//robin::Solver3Lasers mysolver;
//	mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315); //0.105 or 0.160
//	mysolver.setDownsample(0.004f); //dflt=0.005f //Cyl=0.0025f //Cub=0.005f
//	mysolver.setPlaneRemoval(false);
//	//solver.setUseNormals(true);
//	
//	// Dummy Segmentation object
//	//pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>* seg(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);
//	//seg->setNormalDistanceWeight(0.001);
//	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
//	seg->setOptimizeCoefficients(true);
//	seg->setMethodType(pcl::SAC_RANSAC);
//	seg->setMaxIterations(100);
//	seg->setDistanceThreshold(0.001);
//	seg->setRadiusLimits(0.005, 0.050);
//	mysolver.setSegmentation(seg);
//	
//	// Create a sensor from a camera
//	robin::RealsenseD400* mycam(new robin::RealsenseD400());
//	mycam->printInfo();
//	mycam->setDisparity(false);
//	mysolver.addSensor(mycam);
//
//	//robin::LaserArrayCross* myarr(new robin::LaserArrayCross(mycam, 0.001));
//	//mysolver.addSensor(myarr);
//
//	//-----
//	// Create a sensor from another sensor
//	/*robin::LaserScanner* mylaser_h(new robin::LaserScanner(mycam, 0.0, 1.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(mylaser_h);
//	robin::LaserScanner* mylaser_v(new robin::LaserScanner(mycam, 1.0, 0.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(mylaser_v);*/
//
//	/*robin::LaserScanner* laser_0(new robin::LaserScanner(mycam, 0.0, 1.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(laser_0);
//	robin::LaserScanner* laser_1(new robin::LaserScanner(mycam, -1.0, 1.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(laser_1);
//	robin::LaserScanner* laser_2(new robin::LaserScanner(mycam, 1.0, 0.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(laser_2);
//	robin::LaserScanner* laser_3(new robin::LaserScanner(mycam, 1.0, 1.0, 0.0, 0.0, 0.001));
//	mysolver.addSensor(laser_3);*/
//	//-----
//
//	// Create a Primitive
//	//robin::Primitive3Cylinder* prim(new robin::Primitive3Cylinder);
//	//prim->setVisualizeOnOff(false);
//
//	// Create a PCL visualizer
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	int vp(0);
//	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
//	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
//	viewer->setSize(800, 600);
//	float bckgr_gray_level = 0.0;  // Black:=0.0
//	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
//	viewer->addCoordinateSystem(0.25);
//
//	bool RENDER(true);
//	std::vector<double> freq;
//
//	while(true){
//		auto tic = std::chrono::high_resolution_clock::now();
//
//		robin::Primitive3d3* prim(new robin::Primitive3d3);
//		
//		mysolver.solve(prim);
//
//		///
//		controller.evaluate(prim);
//		std::cout << "Grasp_size: " << controller.getGraspSize() << std::endl;
//		std::cout << "Tilt_angle: " << controller.getTiltAngle() << " (" << controller.getTiltAngle() * 180.0 / 3.14159 << ")" << std::endl;
//
//		if (myhand.isRightHand()) {
//			// Right-hand prosthesis (positive tilt angle)
//			std::cout << "Hand grasp_size: " << myhand.getGraspSize() << std::endl;
//			std::cout << "Hand tilt_angle: " << myhand.getWristSupProAngle() << " (" << myhand.getWristSupProAngle() * 180.0 / 3.14159 << ")" << std::endl;
//		} else {
//			// Left-hand prosthesis (negative tilt angle)
//			std::cout << "Hand grasp_size: " << myhand.getGraspSize() << std::endl;
//			std::cout << "Hand tilt_angle: " << -myhand.getWristSupProAngle() << " (" << -myhand.getWristSupProAngle() * 180.0 / 3.14159 << ")" << std::endl;
//		}
//		std::cout << "\n" << std::endl;		
//		///
//
//		//---- RENDERING ----
//		if (RENDER) {
//			// Clean any previously rendered objects
//			viewer->removeAllShapes();
//			viewer->removeAllPointClouds();
//
//			/*for (auto s : mysolver.getSensors()) {
//				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(255, 255, 255);
//				cloud_color_h.setInputCloud(s->getPointCloud());
//				viewer->addPointCloud(s->getPointCloud(), cloud_color_h, std::to_string(std::rand()));
//			}*/
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> preproc_color_h(255, 255, 255);
//			preproc_color_h.setInputCloud(mysolver.getPreprocessed());
//			viewer->addPointCloud(mysolver.getPreprocessed(), preproc_color_h, "preproc");
//
//			///
//
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> solver_color_h(0, 255, 0);
//			solver_color_h.setInputCloud(mysolver.getPointCloud());
//			viewer->addPointCloud(mysolver.getPointCloud(), solver_color_h, "solver");
//			//mysolver.visualize(viewer); //lccp:"marker"
//
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> primitive_color_h(255, 0, 0);
//			primitive_color_h.setInputCloud(prim->getPointCloud());
//			viewer->addPointCloud(prim->getPointCloud(), primitive_color_h, "primitive");
//			prim->visualize(viewer);
//
//			//std::cout << *prim->getCoefficients() << std::endl;
//
//			//------
//			/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(255, 255, 255);
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_color(mycam->getPointCloud());
//			cloud_color_h.setInputCloud(cloud_color);
//			viewer->addPointCloud(cloud_color, cloud_color_h);*/
//			//-----
//
//			viewer->spinOnce(1, true);
//		}
//
//		//---- PROFILING ---
//		auto toc = std::chrono::high_resolution_clock::now();
//		std::chrono::duration<double, std::ratio<1>> t = toc - tic;
//		std::cout << "Cycle duration: " << 1.0/t.count() << " Hz (in " << t.count()*1000.0 << " ms).\n" << std::endl;
//		freq.push_back(t.count());
//	}	
//}




void run_solver(robin::Solver3LCCP*& solver, robin::Primitive3d3* prim);

int main(int argc, char** argv)
{
	// Create a Hand
	robin::hand::Michelangelo myhand(false);

	// Create a Controller
	robin::control::ControlSimple controller(myhand);

	// Pointer to a generic Solver3
	robin::Solver3* solver;

	// Pointer to a generic Primitive3d3
	robin::Primitive3d3* prim;

	// Dummy Segmentation object
	pcl::SACSegmentation<pcl::PointXYZ>* seg(new pcl::SACSegmentation<pcl::PointXYZ>);
	seg->setOptimizeCoefficients(true);
	seg->setMethodType(pcl::SAC_RANSAC);
	seg->setMaxIterations(100);
	seg->setDistanceThreshold(0.001);
	seg->setRadiusLimits(0.005, 0.050);

	// Create a sensor from a camera
	robin::RealsenseD400* mycam(new robin::RealsenseD400());
	mycam->printInfo();

	//////////////////////////////////////////////    SPHERE PRIMITIVE    //////////////////////////////////////////////
	// Declare a Solver3
	robin::Solver3LCCP* solver_sph(new robin::Solver3LCCP);
	solver_sph->setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315); //0.105 or 0.160
	solver_sph->setDownsample(0.0025f); //dflt=0.005f //Cyl=0.0025f //Cub=0.005f
	solver_sph->setPlaneRemoval(false);
	// Add Segmentation object to Solver
	solver_sph->setSegmentation(seg);
	// Add Sensor from a camera to Solver
	solver_sph->addSensor(mycam);
	// Create a Primitive
	robin::Primitive3d3* p_sph = new robin::Primitive3Sphere;

	//////////////////////////////////////////////    CUBOID PRIMITIVE    /////////////////////////////////////////////
	// Declare a Solver3
	robin::Solver3LCCP* solver_cub(new robin::Solver3LCCP);
	solver_cub->setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315); //0.105 or 0.160
	solver_cub->setDownsample(0.005f); //dflt=0.005f //Cyl=0.0025f //Cub=0.005f
	solver_cub->setPlaneRemoval(false);
	// Add Segmentation object to Solver
	solver_cub->setSegmentation(seg);
	// Add Sensor from a camera to Solver
	solver_cub->addSensor(mycam);
	// Create a Primitive
	robin::Primitive3d3* p_cub = new robin::Primitive3Cuboid;

	/////////////////////////////////////////////    CYLINDER PRIMITIVE    ////////////////////////////////////////////
	// Declare a Solver3
	robin::Solver3LCCP* solver_cyl(new robin::Solver3LCCP);
	solver_cyl->setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315); //0.105 or 0.160
	solver_cyl->setDownsample(0.0025f); //dflt=0.005f //Cyl=0.0025f //Cub=0.005f
	solver_cyl->setPlaneRemoval(false);
	// Add Segmentation object to Solver
	solver_cyl->setSegmentation(seg);
	// Add Sensor from a camera to Solver
	solver_cyl->addSensor(mycam);
	// Create a Primitive
	robin::Primitive3d3* p_cyl = new robin::Primitive3Cylinder;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
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
	std::vector<double> freq;

	while (true) {
		auto tic = std::chrono::high_resolution_clock::now();

		std::thread t_sph = std::thread(&run_solver, std::ref(solver_sph), std::ref(p_sph));
		std::thread t_cub = std::thread(&run_solver, std::ref(solver_cub), std::ref(p_cub));
		std::thread t_cyl = std::thread(&run_solver, std::ref(solver_cyl), std::ref(p_cyl));

		t_sph.join();
		t_cub.join();
		t_cyl.join();

		// Pick the biggest fit cloud that corresponds to the correct primitive fitting
		float fit_percent(0.0);

		float sph_lccp_size(solver_sph->getPointCloud()->points.size());
		float sph_fit_size(p_sph->getPointCloud()->points.size());
		std::cout << "Sphere FitPercent: " << sph_fit_size / sph_lccp_size << std::endl;
		if (fit_percent < sph_fit_size / sph_lccp_size) {
			solver = solver_sph;
			prim = p_sph;
		}
		float cub_lccp_size(solver_cub->getPointCloud()->points.size());
		float cub_fit_size(p_cub->getPointCloud()->points.size());
		std::cout << "Sphere FitPercent: " << cub_fit_size / cub_lccp_size << std::endl;
		if (fit_percent < cub_fit_size / cub_lccp_size) {
			solver = solver_cub;
			prim = p_cub;
		}
		float cyl_lccp_size(solver_cyl->getPointCloud()->points.size());
		float cyl_fit_size(p_cyl->getPointCloud()->points.size());
		std::cout << "Sphere FitPercent: " << cyl_fit_size / cyl_lccp_size << std::endl;
		if (fit_percent < cyl_fit_size / cyl_lccp_size) {
			solver = solver_cyl;
			prim = p_cyl;
		}


		///
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
		std::cout << "\n" << std::endl;
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
			preproc_color_h.setInputCloud(solver->getPreprocessed());
			viewer->addPointCloud(solver->getPreprocessed(), preproc_color_h, "preproc");

			///

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> solver_color_h(0, 255, 0);
			solver_color_h.setInputCloud(solver->getPointCloud());
			viewer->addPointCloud(solver->getPointCloud(), solver_color_h, "solver");
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
		std::cout << "Cycle duration: " << 1.0 / t.count() << " Hz (in " << t.count() * 1000.0 << " ms).\n" << std::endl;
		freq.push_back(t.count());
	}
}

void run_solver(robin::Solver3LCCP*& solver, robin::Primitive3d3* prim)
{
	solver->solve(prim);
}