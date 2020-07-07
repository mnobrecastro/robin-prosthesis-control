#include "hand_udp.h"

namespace robin
{
	namespace hand
	{
		HandUDP::HandUDP() {}

		HandUDP::HandUDP(std::string ip, const int port_in, const int port_out)
		{
			this->set_ip_address(ip);
			this->set_port_in(port_in);
			this->set_port_out(port_out);

			
		}

		HandUDP::~HandUDP() {}

		void HandUDP::set_ip_address(std::string ip) { ip_address_ = ip; }

		void HandUDP::set_port_in(int port) { port_in_ = port; }

		void HandUDP::set_port_out(int port) { port_out_ = port; }
	}
}