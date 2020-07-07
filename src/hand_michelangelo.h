#pragma once
#include "hand_udp.h"

#define IP_ADDRESS "127.0.0.1"
#define PORT_IN 8052
#define PORT_OUT 8051

namespace robin
{
	namespace hand
	{
		class Michelangelo :
			public HandUDP
		{
		public:
			Michelangelo();
			Michelangelo(std::string ip, const int port_in, const int port_out);
			~Michelangelo();

			/* Unavailable Commands */
			void flex() = delete;
			void extend() = delete;
			void abduct() = delete;
			void adduct() = delete;

			void pronate();
			void supinate();

			void open();
			void close();

		private:
			void grasp_palmar();
			void grasp_lateral();
		};
	}
}