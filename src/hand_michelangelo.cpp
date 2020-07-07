#include "hand_michelangelo.h"

namespace robin
{
	namespace hand
	{
		Michelangelo::Michelangelo()
		{
			HandUDP(IP_ADDRESS, PORT_IN, PORT_OUT);
		}

		Michelangelo::Michelangelo(std::string ip, const int port_in, const int port_out)
		{
			HandUDP(ip, port_in, port_out);
		}

		Michelangelo::~Michelangelo() {}


	}
}