#pragma once
#include "hand_udp.h"
// ---- dynamic_cast
#include "solver1_emg.h"
// ----
#include "data_manager.h"

#include <algorithm>
#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>

#include "../dependencies/gnuplot-iostream/gnuplot-iostream.h"

#define IP_ADDRESS "127.0.0.1"
#define PORT_IN 8052
#define PORT_OUT 8051
#define PACKET_IN_LENGTH 35
#define PACKET_OUT_LENGTH 9

#define EMG_CHANNELS 8

#define M_PI	3.14159265358979323846   // pi

namespace robin
{
	namespace hand
	{
		class Michelangelo :
			public HandUDP
		{
		public:
			Michelangelo() = delete;
			Michelangelo(bool right_hand);
			Michelangelo(bool right_hand, const char* ip, short port_in, short port_out);
			~Michelangelo();

			void plotEMG(bool b=false) { plotEMGOnOff_ = b; }

			/* Calibrate the EMG channels (ex. normalize to MVC). */
			void calibrateEMG();

			/* Set a configuration state for the prosthesis. */
			void setConfigState() {}

			/* Read the current configuration state of the prosthesis. */
			std::vector<float> getConfigState() { return std::vector<float>(); }

			/* Prosthesis config state accessors. */
			float getWristFleExtAngle();
			float getWristAbdAddAngle();
			float getWristSupProAngle();
			float getGraspSize();
			float getGraspForce();
			std::vector<float> getEMG();
			
			std::vector<Solver1EMG*> getEMGSolvers() const { return emg_solvers_; }

			/* Prothesis commands available. */
			void pronate(float vel, bool send = true);
			void supinate(float vel, bool send = true);
			void open(GRASP g, float vel, bool send = true);
			void close(GRASP g, float vel, bool send = true);
			void stop();

			void send_command();

			void setDataManager(robin::data::DataManager& dm);

		protected:
			/* Array to keep track of the command variables */
			std::array<float, 9> command_buffer_ = { 1 / 255, 0.0,0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0 };

			/* Receive sensor data from the hand.
			 *
			 * Only supported 'Dump Mode' = "100Hz, EMG + Sensors". In this case, the received
			 * data packet is of 35 bytes, with the following scheme: (full details in the readme)
			 *
			 * byte00 : uint8, main drive
			 * byte01 : uint8, thumb drive
			 * byte02 : uint8, rotation angle
			 * byte03 : uint8, flexion angle
			 * byte04 : uint8, force
			 * byte05 : bool,  grasp type
			 * byte06 : int8,  aperture [-100, 100] #TODO: doesn't sound right!
			 * byte07 : int8,  pronation/supination angle [-100, 100]
			 * byte08 : int8,  flexion/extension [-100, 100]
			 * byte09 : int8,  force [-100, 100] #TODO: doesn't make sense!
			 * byte10 - byte35 : Controller state, EMG data and overhead. */
			int receive_packet(uint8_t packet[]);

			/* Send velocity commands to the prosthesis.
			 *
			 * Only supported control mode: `Velocity control`. Data packet to be sent should have
			 * 9 bytes all of type uint8, in the following scheme:
			 *
			 * byte0 = 1: to indicate velocity-control mode
			 * byte1 = uint8: Palmar Grip Closing command in range [0, 255]
			 * byte2 = uint8: Palmar Grip Opening command in range [0, 255]
			 * byte3 = uint8: Lateral Grip Closing command in range [0, 255]
			 * byte4 = uint8: Lateral Grip Opening command in range [0, 255]
			 * byte5 = uint8: Pronation Velocity in range [0, 255]
			 * byte6 = uint8: Supination Velocity in range [0, 255]
			 * byte7 = uint8: Flexion Velocity in range [0, 255]
			 * byte8 = uint8: Extension Velocity in range [0, 255]
			 *
			 * Input Arguments
			 * ---------
			 * control_command : array_like
			 *	8-dimensional array of speeds for the different signals */
			void send_packet(const uint8_t* packet, size_t packet_byte_length);					

			/* Print a received packet (variables according to the manufacturer). */
			void print_recv_packet(const uint8_t* packet, size_t packet_byte_length);

			//void grasp_palmar();
			//void grasp_lateral();

			std::vector<Solver1EMG*> emg_solvers_;			

		private:
			/* Suppressed Commands */
			using Hand::flex;
			using Hand::extend;
			using Hand::abduct;
			using Hand::adduct;

			bool is_moving_ = false;
			bool is_dumping_ = false;
			std::thread thread_configstate_;
			std::mutex mu_configstate_;
			std::thread thread_emgproc_;
			std::mutex mu_emgproc_;
			
			/* Function to be called by thread_configstate_ */
			void updateConfigState();

			void feedChildren();

			/* Function to be called by thread_emgproc_ */
			void updateEMG();

			bool calibratedOnOff_ = false;
			bool plotEMGOnOff_ = false;

			// Create a Gnuplot canvas
			Gnuplot gp_;
			std::vector<std::pair<double, double>> gnup_emg0_, gnup_emg1_, gnup_emg2_, gnup_emg3_, gnup_emg4_, gnup_emg5_, gnup_emg6_, gnup_emg7_;
			std::size_t kdata_ = 0;
			
			float dummy = 0.05;



			/* Declaration of a DataManager object. */
			robin::data::DataManager* dm_ = nullptr;
			int saveCalibration(const std::vector<float>& baseval, const std::vector<float>& normval) const;
			int loadCalibration(std::vector<float>& baseval, std::vector<float>& normval) const;
		};
	}
}