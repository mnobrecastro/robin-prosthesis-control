#pragma once
#include "hand_udp.h"

#define IP_ADDRESS "127.0.0.1"
#define PORT_IN 8052
#define PORT_OUT 8051
#define PACKET_IN_LENGTH 35
#define PACKET_OUT_LENGTH 9

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

			/* Set a configuration state for the prosthesis. */
			void setConfigState() {}

			/* Read the current configuration state of the prosthesis. */
			std::vector<float> getConfigState() { return std::vector<float>(); }

			void pronate(float vel) {}
			void supinate(float vel) {}

			void open(float vel) {}
			void close(float vel) {}

		protected:
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
			size_t receive_packet(uint8_t packet[]);

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

			void updateConfigState();

			//void grasp_palmar();
			//void grasp_lateral();

		private:
			/* Suppressed Commands */
			using Hand::flex;
			using Hand::extend;
			using Hand::abduct;
			using Hand::adduct;
		};
	}
}