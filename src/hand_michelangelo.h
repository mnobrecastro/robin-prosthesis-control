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
			Michelangelo(bool right_hand, const char* ip, short port_in, short port_out);
			~Michelangelo();

			/* Set a configuration state for the prosthesis. */
			void set_state();

			/* Read the current configuration state of the prosthesis. */
			void get_state();

			/* Suppressed Commands */
			/*void flex() = delete;
			void extend() = delete;
			void abduct() = delete;
			void adduct() = delete;*/

			/*void pronate();
			void supinate();

			void open();
			void close();*/

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
			uint8_t* receive_packet();

			/* Protect the capability of sending an arbitrarily sized packet. */
			//using HandUDP::send_packet;

			/* Send velocity commands to the prosthesis.
			 *
			 * Only supported control mode: `Velocity control`. Data packet to be sent should have
			 * 9 bytes all of type uint8, in the following scheme:
			 *
			 * byte00 = 1,     velocity mode indicator
			 * byte01 : uint8, palmar grip closing [0, 255]
			 * byte02 : uint8, palmar grip opening [0, 255]
			 * byte03 : uint8, lateral grip closing [0, 255]
			 * byte04 : uint8, lateral grip opening [0, 255]
			 * byte05 : uint8, pronation velocity [0, 255]
			 * byte06 : uint8, supination velocity [0, 255]
			 * byte07 : uint8, flexion velocity [0, 255]
			 * byte08 : uint8, extension velocity [0, 255]

			 * Input Arguments
			 * ---------
			 * control_command : array_like
			 *		8-dimensional array of speeds for the different signals */
			void send_packet(const uint8_t* packet, size_t packet_byte_length);

			//void grasp_palmar();
			//void grasp_lateral();
		};
	}
}