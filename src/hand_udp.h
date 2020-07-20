#pragma once
#include "hand.h"

#include <stdlib.h>
#include <iostream>
#include <array>
#include <string>
#include <iomanip>

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

namespace robin
{
	namespace hand
	{
		class HandUDP :
			public Hand
		{
		public:
			HandUDP() = delete;
			HandUDP(bool right_hand, const char* ip, short port_in, short port_out);
			~HandUDP();

			/**/
			void set_ip_address(const char* ip);

			/**/
			void set_port_in(short port);

			/**/
			void set_port_out(short port);

			/* Receive a packet of data from the prosthesis
			* (Subclasses of HandUDP should not have access to this member function since it is meant to be "developers only")  */
			virtual size_t receive_packet(uint8_t packet[]);

			/* Send a packet of data to the prosthesis
			 * (Subclasses of HandUDP should not have access to this member function since it is meant to be "developers only")  */
			virtual void send_packet(const uint8_t* packet, size_t packet_byte_length);

			/* Print a received packet. */
			virtual void print_recv_packet(const uint8_t* packet, size_t packet_byte_length);

		protected:
			const char* ip_address_; // IPv4
			short port_in_; // Receiver Port
			short port_out_; // Transmission Port
			size_t packet_in_length_;
			size_t packet_out_length_;

			void virtual set_packet_in_length(size_t n) {}
			void virtual set_packet_out_length(size_t n) {}

			// Winsocket variables
			const char* local_ip_;
			const char* dest_ip_;
			sockaddr_in local_ipv4_; //IPv4 Socket address (Internet style)
			sockaddr_in dest_ipv4_; //IPv4 Socket address (Internet style)
			WSAData wsadata_;
			SOCKET socket_;

			/* Opens/initializes a socket for UDP communication with the hand prosthesis. */
			void open_socket();

			/* Closes the socket for UDP communication. */
			void close_socket();

		};
	}
}