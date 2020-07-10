#include "hand_udp.h"

namespace robin
{
	namespace hand
	{
		HandUDP::HandUDP() {}

		HandUDP::HandUDP(const char* ip, short port_in, short port_out)
		{
			this->set_ip_address(ip);
			this->set_port_in(port_in);
			this->set_port_out(port_out);
			std::cout << "IP address: " << ip_address_ << " (in:" << port_in_ << ",out:" << port_out_ << ")" << std::endl;

			this->open_socket();
		}

		HandUDP::~HandUDP()
		{
			this->close_socket();
		}

		void HandUDP::set_ip_address(const char* ip) { ip_address_ = ip; }

		void HandUDP::set_port_in(short port) { port_in_ = port; }

		void HandUDP::set_port_out(short port) { port_out_ = port; }

		/* Opens/initializes a socket for UDP communication with the hand prosthesis. */
		void HandUDP::open_socket()
		{
			// Initialize Winsocket variables
			local_ip_ = ip_address_;
			dest_ip_ = ip_address_;
			WSAStartup(MAKEWORD(2, 2), &wsadata_);

			// Receiver/server socket
			local_ipv4_.sin_family = AF_INET;
			inet_pton(AF_INET, local_ip_, &local_ipv4_.sin_addr.s_addr);
			local_ipv4_.sin_port = htons(port_in_);

			// Sender/client socket
			dest_ipv4_.sin_family = AF_INET;
			inet_pton(AF_INET, dest_ip_, &dest_ipv4_.sin_addr.s_addr);
			dest_ipv4_.sin_port = htons(port_out_);

			socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			bind(socket_, (sockaddr*)&local_ipv4_, sizeof(local_ipv4_));
		}

		/* Closes the socket for UDP communication. */
		void HandUDP::close_socket()
		{
			closesocket(socket_);
			WSACleanup();
		}

		/* Receives a packet of data from the prosthesis. */
		uint8_t* HandUDP::receive_packet()
		{
			uint8_t packet[1024];
			int local_addr_size = sizeof(local_ipv4_);
			int nbytes = recvfrom(socket_, (char*)packet, sizeof(packet[0]) * (sizeof(packet) / sizeof(uint8_t)), 0, (sockaddr*)&local_ipv4_, &local_addr_size);
			if (nbytes <= 1) {
				std::cerr << "The UDP connection failed to receive a packet." << std::endl;
				return nullptr;
			}
			else {
				return packet;
			}
		}

		/* Closes the socket for UDP communication. */
		void HandUDP::send_packet(uint8_t* packet)
		{
			int dest_addr_size = sizeof(dest_ipv4_);
			int nbytes = sendto(socket_, (char*)packet, sizeof(packet[0]) * (sizeof(packet) / sizeof(uint8_t)), 0, (sockaddr*)&dest_ipv4_, dest_addr_size);
			if (nbytes <= 1) {
				std::cerr << "The UDP connection failed to send a packet." << std::endl;
				return;
			}
		}
	}
}