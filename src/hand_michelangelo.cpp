#include "hand_michelangelo.h"

namespace robin
{
	namespace hand
	{

		Michelangelo::Michelangelo(bool right_hand)
			: HandUDP(right_hand, IP_ADDRESS, PORT_IN, PORT_OUT)
		{
			this->updateConfigState();
			do {
				std::cout << '\n' << "Press any key to continue...";
			} while (std::cin.get() != '\n');
		}
		
		Michelangelo::Michelangelo(bool right_hand, const char* ip, short port_in, short port_out)
			: HandUDP(right_hand, ip, port_in, port_out)
		{
			this->updateConfigState();
			do {
				std::cout << '\n' << "Press any key to continue...";
			} while (std::cin.get() != '\n');
		}

		Michelangelo::~Michelangelo() {}

		/* Prothesis commands available. */
		/* byte0 = 1: to indicate velocity - control mode
		 * byte1 = uint8 : Palmar Grip Closing command in range[0, 255]
		 * byte2 = uint8 : Palmar Grip Opening command in range[0, 255]
		 * byte3 = uint8 : Lateral Grip Closing command in range[0, 255]
		 * byte4 = uint8 : Lateral Grip Opening command in range[0, 255]
		 * byte5 = uint8 : Pronation Velocity in range[0, 255]
		 * byte6 = uint8 : Supination Velocity in range[0, 255]
		 * byte7 = uint8 : Flexion Velocity in range[0, 255]
		 * byte8 = uint8 : Extension Velocity in range[0, 255]*/
		void Michelangelo::pronate(float vel)
		{
			/* byte5 = uint8 : Pronation Velocity in range[0, 255]
			 * byte6 = uint8 : Supination Velocity in range[0, 255]
			 */
			command_buffer_[5] = min(max(0.0, std::abs(vel)), 1.0);
			command_buffer_[6] = 0.0;
			this->send_command();
		}
		void Michelangelo::supinate(float vel)
		{
			/* byte5 = uint8 : Pronation Velocity in range[0, 255]
			 * byte6 = uint8 : Supination Velocity in range[0, 255]
			 */
			command_buffer_[5] = 0.0;
			command_buffer_[6] = min(max(0.0, std::abs(vel)), 1.0);
			this->send_command();
		}

		void Michelangelo::open(float vel)
		{
			//command_buffer_
		}
		void Michelangelo::close(float vel)
		{
			//command_buffer_
		}


		/**** PROTECTED MEMBER FUNCTIONS ****/

		size_t Michelangelo::receive_packet(uint8_t packet[])
		{
			size_t packet_byte_length = HandUDP::receive_packet(packet);
			if (packet_byte_length != PACKET_IN_LENGTH) {
				std::cerr << "Problem receiving a packet." << std::endl;
				return -1;
			}
			else {
				return packet_byte_length;
			};
		}

		void Michelangelo::send_packet(const uint8_t* packet, size_t packet_byte_length)
		{
			if (packet_byte_length != PACKET_OUT_LENGTH) {
				std::cerr << "Wrong packet size. Correct size is " << PACKET_OUT_LENGTH << " bytes." << std::endl;
			}
			else {
				HandUDP::send_packet(packet, packet_byte_length);
			}
		}

		void Michelangelo::send_command()
		{
			uint8_t packet[PACKET_OUT_LENGTH];
			size_t packet_byte_length = sizeof(packet)/sizeof(uint8_t);

			for (int i(0); i < packet_byte_length; ++i) {
				*(packet + i) = int(command_buffer_[i] * 255);
				std::cout << command_buffer_[i] << " (" << +*(packet + i) << ") ";
			}
			std::cout << std::endl;
			
			this->send_packet(packet, packet_byte_length);
		}

		void Michelangelo::print_recv_packet(const uint8_t* packet, size_t packet_byte_length)
		{
			const uint8_t* packet_;
			packet_ = packet;
			
			if (packet_byte_length != PACKET_IN_LENGTH) {
				std::cerr << "Wrong packet size. Correct size is " << PACKET_IN_LENGTH << " bytes." << std::endl;
			}
			else {
				for (int i(0); i < packet_byte_length; ++i) {
					// '+' is a prefix operator to print the actual number instead of char
					std::cout << +*(packet_ + i) << " ";
				}
				std::cout << std::endl;


				std::cout << "Raw sensor data:" << std::endl;
				for (int i(0); i < 5; ++i) {
					std::cout << +*(packet_ + i) << " ";
				}
				std::cout << std::endl;

				std::cout << "Normalized sensor data (human readable):" << std::endl;
				std::cout << (int)+*(packet_ + 5) << " ";
				for (int i(6); i < 10; ++i) {
					std::cout << +int8_t(*(packet_ + i)) << " ";
				}
				std::cout << std::endl;

				std::cout << "Controller state (when in 'Co-Contraction Mode'):" << std::endl;
				for (int i(10); i < 14; i += 2) {
					uint8_t most_significant_byte = *(packet_ + i);
					uint8_t least_significant_byte = *(packet_ + i + 1);
					uint16_t val = (most_significant_byte << 8) | least_significant_byte; // Big-Endian
					std::cout << +val << " ";
				}
				std::cout << (int)+*(packet_ + 14) << " ";
				std::cout << std::endl;

				std::cout << "Raw EMG data:" << std::endl;
				for (int i(15); i < 31; i += 2) {
					uint8_t most_significant_byte = *(packet_ + i);
					uint8_t least_significant_byte = *(packet_ + i + 1);
					uint16_t val = (most_significant_byte << 8) | least_significant_byte; // Big-Endian
					std::cout << +val << " ";
				}
				std::cout << std::endl;

				std::cout << "Overhead for data management:" << std::endl;
				for (int i(32); i < 35; ++i) {
					std::cout << +*(packet_ + i) << " ";
				}
				std::cout << std::endl;
			}
		}

		void Michelangelo::updateConfigState()
		{
			uint8_t packet[1024];
			size_t byte_length(this->receive_packet(packet));

			// byte5: "Grasp Type" (int8), 0 for palmar and 1 for lateral
			int grasp_type = +*(packet + 5);
			if (grasp_type == 0) { configstate_.grasp_type = GRASP::PALMAR; }
			else if (grasp_type == 1) { configstate_.grasp_type = GRASP::LATERAL; }

			// byte6: "Aperture" (int8), range [0,100]%
			// -> Palmar = [0.000,0.110]m
			// -> Lateral = [0.000,0.070]m
			int grasp_size = +int8_t(*(packet + 6));
			switch (configstate_.grasp_type) {
			case GRASP::PALMAR:
				configstate_.grasp_size = grasp_size/100 * 0.110;
				break;
			case GRASP::LATERAL:
				configstate_.grasp_size = grasp_size/100 * 0.070;
				break;
			}

			// byte7: "Pronation/Supination" (int8), range [-100,100]% -> [-160,160]deg
			int sup_pro_angle = +int8_t(*(packet + 7));
			configstate_.sup_pro_angle = float(sup_pro_angle/100.0 * 160*M_PI/180);
			std::cout << "ConfigState Sup/Pro: " << configstate_.sup_pro_angle << std::endl;

			// byte8: "Flexion/Extension" (int8), range [-100,100]% -> [-45,75]deg
			int fle_ext_angle = +int8_t(*(packet + 8));
			configstate_.fle_ext_angle = fle_ext_angle; // :=0.0

			// byte9: "Force" (int8), range [0, 100]%
			// -> Gripping force in Opposition/Palmar Mode = 70 N (commercial) or 100 N (research)
			// -> Gripping force in Lateral Mode = 60 N
			// -> Gripping force in Neutral Mode = 15 N
			int  grasp_force = +int8_t(*(packet + 9));
			switch (configstate_.grasp_type) {
			case GRASP::PALMAR:
				configstate_.grasp_force = grasp_force/100 * 100;
				break;
			case GRASP::LATERAL:
				configstate_.grasp_force = grasp_force/100 * 60;
				break;
			}
			
		}
	}
}