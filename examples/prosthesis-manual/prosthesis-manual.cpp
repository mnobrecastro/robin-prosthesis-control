//#define MULTITHREADING
//#define GNUPLOT

#include <robin/utils/data_manager.h>
#include <robin/sensor/hand_michelangelo.h>
#include <robin/control/control_simple.h>

#include <robin/primitive/primitive3.h>

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
	myhand.setDataManager(mydm);
	myhand.plotEMG(false);
	myhand.calibrateEMG();

	//robin::hand::Hand myhand(TRUE);

	robin::control::ControlSimple controller(myhand);
	controller.setFilter(robin::control::ControlVar::fname::MOVING_AVERAGE, 20); //20=~200ms     //MEDIAN, 40
	controller.setFullManual(false);
	controller.setDataManager(mydm);


	// Create a Primitive
	robin::Primitive3d3* prim;

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


		if (HAND_CONTROL) {
			controller.evaluate(prim);
			std::cout << "Grasp_size: " << controller.getGraspSize() << std::endl;
			std::cout << "Tilt_angle: " << controller.getTiltAngle() << " (" << controller.getTiltAngle() * 180.0 / 3.14159 << ")" << std::endl;
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