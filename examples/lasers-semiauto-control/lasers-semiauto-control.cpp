//#define MULTITHREADING
//#define GNUPLOT

#include <robin/utils/data_manager.h>
#include <robin/sensor/hand_michelangelo.h>
//#include <robin/control/control_simple.h>
#include <robin/control/control_sequential.h>
#include <robin/solver/solver3_lasers.h>
#include <robin/sensor/realsense_d400.h>
#include <robin/sensor/laser_array.h>
#include <robin/primitive/primitive3.h>
#include <robin/feedback/feedback.h>
#include <robin/sensor/engacoustics_tactor.h>

#include <chrono>
#include <thread>

#ifdef GNUPLOT
#include "gnuplot-iostream/gnuplot-iostream.h"
#endif


pcl::PointCloud<pcl::PointXYZ>::Ptr rotate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, Eigen::Vector3f axis, float angle);


int main(int argc, char** argv)
{
	// ---
	robin::data::DataManager mydm;
	// ---
	
	Beep(523, 100); 

	robin::hand::Michelangelo myhand(false);
	myhand.setDataManager(mydm);
	myhand.plotEMG(false);
	myhand.calibrateEMG();

	robin::control::ControlSequential controller(myhand);
	controller.setFilter(robin::control::ControlVar::fname::MOVING_AVERAGE, 20); //20=~200ms     //MEDIAN, 40   // <--- Not being used at the moment
	controller.setFullManual(false);
	controller.setDataManager(mydm);

	// Declare a solver3
	robin::Solver3Lasers mysolver;
	//mysolver.setCrop(-0.1, 0.1, -0.1, 0.1, 0.115, 0.315);
	mysolver.setCrop(-0.05, 0.05, -0.05, 0.05, 0.115, 0.215);
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
	robin::LaserArraySingle* myarr(new robin::LaserArraySingle(mycam, 0.002));
	//robin::LaserArrayCross* myarr(new robin::LaserArrayCross(mycam, 0.002));
	//robin::LaserArrayStar* myarr(new robin::LaserArrayStar(mycam, 0.001));
	mysolver.addSensor(myarr);

	// Create a Primitive
	robin::Primitive3d3* prim;


	// Create a Feedback object and add a Tactor instance to it
	robin::feedback::Feedback feed;
	robin::EngAcousticsTactor* tactor(new robin::EngAcousticsTactor({ 3, 4, 5, 6 }, 0.7, "COM6"));
	feed.addTactor(tactor);


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
	bool FEEDBACK(true);
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
		
		if (FEEDBACK) {
			if (myhand.isRightHand())
				feed.addPointCloud(rotate(mysolver.getPointCloud(), { 0.0, 0.0, 1.0 }, myhand.getWristSupProAngle()), robin::FEEDBACK_CLOUD::DIST_TO_HULL);
			else
				feed.addPointCloud(rotate(mysolver.getPointCloud(), { 0.0, 0.0, 1.0 }, -myhand.getWristSupProAngle()), robin::FEEDBACK_CLOUD::DIST_TO_HULL);
			feed.addPrimitive3(prim, robin::FEEDBACK_PRIM::TYPE);
			feed.run();
		}

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

			if (FEEDBACK) {
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> feed_color_h(255, 255, 255);
				feed_color_h.setInputCloud(feed.getPointCloud());
				viewer->addPointCloud(feed.getPointCloud(), feed_color_h, "feed");
				feed.visualize(viewer);
			}

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



/* Rotates a PointCloud about a given axis according to Euler's rotation theorem. */
pcl::PointCloud<pcl::PointXYZ>::Ptr rotate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, Eigen::Vector3f axis, float angle)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rot(new pcl::PointCloud<pcl::PointXYZ>);
	axis.normalize();

	// Calculate the Euler parameters for rotating a given point about the 'axis' by an amount of 'angle' 
	float e0(std::cos(angle / 2.0));
	Eigen::Vector3f e = axis * std::sin(angle / 2.0);
	Eigen::Matrix3f e_skewsim;
	e_skewsim << 0.0, -e(2), e(1),
		e(2), 0.0, -e(0),
		-e(1), e(0), 0.0;
	Eigen::Matrix3f A = (2 * e0 * e0 - 1) * Eigen::Matrix3f::Identity() + 2 * (e * e.transpose() + e0 * e_skewsim);

	// Rotate the PointCloud
	for (auto p : cloud->points) {
		Eigen::Vector3f p_rot = A * Eigen::Vector3f(p.x, p.y, p.z);
		cloud_rot->points.push_back(pcl::PointXYZ(p_rot.x(), p_rot.y(), p_rot.z()));
	}

	return cloud_rot;
}