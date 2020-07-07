#pragma once
#include "hand.h"

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

namespace robin
{
	namespace hand
	{
		class HandUDP
		{
		public:
			HandUDP();
			HandUDP(std::string ip, int port_in, int port_out);
			~HandUDP();

			void set_ip_address(std::string ip);

			void set_port_in(const int port);

			void set_port_out(const int port);

		protected:
			std::string ip_address_; // IPv4
			int port_in_; // Receiver Port 
			int port_out_; // Transmission Port
		};
	}
}