#include "hand_michelangelo.h"

namespace robin
{
	namespace hand
	{

		Michelangelo::Michelangelo(bool right_hand, const char* ip=IP_ADDRESS, short port_in=PORT_IN, short port_out=PORT_OUT)
			: HandUDP(right_hand, ip, port_in, port_out) {}

		Michelangelo::~Michelangelo() {}

		/* Set a configuration state for the prosthesis. */
		void Michelangelo::set_state()
		{
			
		}

		/* Read the current configuration state of the prosthesis. */
		void Michelangelo::get_state() {

		}

		uint8_t* Michelangelo::receive_packet()
		{
			uint8_t* packet_temp = HandUDP::receive_packet();
			if (packet_temp == nullptr) {
				std::cerr << "Problem receiving a packet." << std::endl;
				return nullptr;
			}
			else {
				uint8_t packet[PACKET_IN_LENGTH];
				for (size_t i(0); i < PACKET_IN_LENGTH; ++i) {
					packet[i] = packet_temp[i];
				}
				return packet;
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

	}
}