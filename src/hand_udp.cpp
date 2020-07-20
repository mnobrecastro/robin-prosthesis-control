#include "hand_udp.h"

namespace robin
{
	namespace hand
	{
		HandUDP::HandUDP(bool right_hand, const char* ip, short port_in, short port_out) :
			Hand(right_hand), ip_address_(ip), port_in_(port_in), port_out_(port_out)
		{
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
			local_ipv4_.sin_port = htons(port_in_); //htons, converts a 16-bit quantity from host byte order to network byte order (Little-Endian to Big-Endian).

			// Sender/client socket
			dest_ipv4_.sin_family = AF_INET;
			inet_pton(AF_INET, dest_ip_, &dest_ipv4_.sin_addr.s_addr);
			dest_ipv4_.sin_port = htons(port_out_); //htons, converts a 16-bit quantity from host byte order to network byte order (Little-Endian to Big-Endian).

			socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			bind(socket_, (sockaddr*)&local_ipv4_, sizeof(local_ipv4_));

			std::cout << "A socket was oppened at " << ip_address_ << " (in:" << port_in_ << ",out:" << port_out_ << ")" << std::endl;
		}

		/* Closes the socket for UDP communication. */
		void HandUDP::close_socket()
		{
			closesocket(socket_);
			WSACleanup();
		}

		/* Receives a packet of data from the prosthesis. */
		size_t HandUDP::receive_packet(uint8_t packet[])
		{
			uint8_t packet_temp[1024];
			size_t byte_length_temp(sizeof(packet_temp) / sizeof(uint8_t));
			int local_addr_size = sizeof(local_ipv4_);
			size_t packet_byte_length = recvfrom(socket_, (char*)packet_temp, byte_length_temp, 0, (sockaddr*)&local_ipv4_, &local_addr_size);
			if (packet_byte_length <= 1) {
				std::cerr << "The UDP connection failed to receive a packet." << std::endl;
				return -1;
			}
			else {
				std::cout << "A packet (" << packet_byte_length << " bytes) was received through UDP connection." << std::endl;
				this->print_recv_packet(packet_temp, packet_byte_length);
				for (int i(0); i < packet_byte_length; ++i) {
					*(packet + i) = *(packet_temp + i);
				}
				return packet_byte_length;
			}
		}

		/* Sends a packet of data to the prosthesis. */
		void HandUDP::send_packet(const uint8_t* packet, size_t packet_byte_length)
		{
			int dest_addr_size = sizeof(dest_ipv4_);
			int n_bytes = sendto(socket_, (char*)packet, packet_byte_length, 0, (sockaddr*)&dest_ipv4_, dest_addr_size);
			if (n_bytes < 1) {
				std::cerr << "The UDP connection failed to send a packet." << std::endl;
				return;
			}
			else {
				std::cout << "A packet (" << n_bytes << " bytes) was sent through UDP connection." << std::endl;
			}
		}

		/* Print a received packet. */
		void HandUDP::print_recv_packet(const uint8_t* packet, size_t packet_byte_length)
		{
			const uint8_t* packet_;
			packet_ = packet;

			for (int i(0); i < packet_byte_length; ++i) {
				// '+' is a prefix operator to print the actual number instead of char
				std::cout << +*(packet_ + i) << " ";
			}
			std::cout << std::endl;
		}
	}
}