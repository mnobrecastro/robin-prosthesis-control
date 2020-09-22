#include "hand_michelangelo.h"

namespace robin
{
	namespace hand
	{

		Michelangelo::Michelangelo(bool right_hand, const char* ip, short port_in, short port_out)
			: HandUDP(right_hand, ip, port_in, port_out)
		{
			configstate_.emg_channels.push_back(0.0); //0
			configstate_.emg_channels.push_back(0.0); //1
			configstate_.emg_channels.push_back(0.0); //2
			configstate_.emg_channels.push_back(0.0); //3
			configstate_.emg_channels.push_back(0.0); //4
			configstate_.emg_channels.push_back(0.0); //5
			configstate_.emg_channels.push_back(0.0); //6
			configstate_.emg_channels.push_back(0.0); //7

			thread_configstate_ = std::thread(&Michelangelo::updateConfigState, this);
		}

		Michelangelo::Michelangelo(bool right_hand)
			: Michelangelo(right_hand, IP_ADDRESS, PORT_IN, PORT_OUT) {}

		Michelangelo::~Michelangelo() {}


		float Michelangelo::getWristFleExtAngle()
		{
			mu_configstate_.lock();
			float val(configstate_.fle_ext_angle);
			mu_configstate_.unlock();
			return val;
		}

		float Michelangelo::getWristAbdAddAngle()
		{
			mu_configstate_.lock();
			float val(configstate_.abd_add_angle);
			mu_configstate_.unlock();
			return val;
		}

		float Michelangelo::getWristSupProAngle()
		{
			mu_configstate_.lock();
			float val(configstate_.sup_pro_angle);
			mu_configstate_.unlock();
			return val;
		}

		float Michelangelo::getGraspSize()
		{
			mu_configstate_.lock();
			float val(configstate_.grasp_size);
			mu_configstate_.unlock();
			return val;
		}

		float Michelangelo::getGraspForce()
		{
			mu_configstate_.lock();
			float val(configstate_.grasp_force);
			mu_configstate_.unlock();
			return val;
		}

		std::vector<float> Michelangelo::getEMG()
		{
			mu_configstate_.lock();
			std::vector<float> emg(configstate_.emg_channels);
			mu_configstate_.unlock();
			return emg;
		}

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

		void Michelangelo::pronate(float vel, bool send)
		{
			if (is_dumping_) {
				/* byte5 = uint8 : Pronation Velocity in range[0, 255]
				 * byte6 = uint8 : Supination Velocity in range[0, 255]
				 */
				command_buffer_[5] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
				command_buffer_[6] = 0.0;
				if (send) {
					this->send_command();
					is_moving_ = true;
				}
			}
		}
		void Michelangelo::supinate(float vel, bool send)
		{
			if (is_dumping_) {
				/* byte5 = uint8 : Pronation Velocity in range[0, 255]
				 * byte6 = uint8 : Supination Velocity in range[0, 255]
				 */
				command_buffer_[5] = 0.0;
				command_buffer_[6] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
				if (send) {
					this->send_command();
					is_moving_ = true;
				}
			}
		}

		void Michelangelo::open(float vel, bool send)
		{
			if (is_dumping_) {
				/* byte1 = uint8 : Palmar Grip Closing command in range[0, 255]
				 * byte2 = uint8 : Palmar Grip Opening command in range[0, 255]
				 * byte3 = uint8 : Lateral Grip Closing command in range[0, 255]
				 * byte4 = uint8 : Lateral Grip Opening command in range[0, 255]
				 */
				/*if (command_buffer_[1] == 0.0 && command_buffer_[2] == 0.0 && command_buffer_[3] == 0.0 && command_buffer_[4] == 0.0) {
					command_buffer_[1] = 0.0;
					command_buffer_[2] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
					command_buffer_[3] = 0.0;
					command_buffer_[4] = 0.0;
					std::cout << "O0O0O0O0O0O0O0O0O0O0O0O0O0O0O0" << std::endl;
				}
				else if (command_buffer_[1] > 0.0 || command_buffer_[2] > 0.0) {
					command_buffer_[1] = 0.0;
					command_buffer_[2] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
					command_buffer_[3] = 0.0;
					command_buffer_[4] = 0.0;
					std::cout << "O1O1O1O1O1O1O1O1O1O1O1O1O1O1O1" << std::endl;
				}
				else if(command_buffer_[3] > 0.0 || command_buffer_[4] > 0.0){
					command_buffer_[1] = 0.0;
					command_buffer_[2] = 0.0;
					command_buffer_[3] = 0.0;
					command_buffer_[4] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
					std::cout << "O2O2O2O2O2O2O2O2O2O2O2O2O2O2O2" << std::endl;
				}*/

				command_buffer_[1] = 0.0;
				command_buffer_[2] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
				command_buffer_[3] = 0.0;
				command_buffer_[4] = 0.0;
				if (send) {
					this->send_command();
					is_moving_ = true;
				}
			}
		}
		void Michelangelo::close(GRASP g, float vel, bool send)
		{
			if (is_dumping_) {
				/* byte1 = uint8 : Palmar Grip Closing command in range[0, 255]
				 * byte2 = uint8 : Palmar Grip Opening command in range[0, 255]
				 * byte3 = uint8 : Lateral Grip Closing command in range[0, 255]
				 * byte4 = uint8 : Lateral Grip Opening command in range[0, 255]
				 */
				switch (g) {
				case GRASP::PALMAR:
					command_buffer_[1] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
					command_buffer_[2] = 0.0;
					command_buffer_[3] = 0.0;
					command_buffer_[4] = 0.0;
					std::cout << "PPPPPPPPPPPPPPPPPPPPPPPPPPPPP" << std::endl;
					break;
				case GRASP::LATERAL:
					command_buffer_[1] = 0.0;
					command_buffer_[2] = 0.0;
					command_buffer_[3] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
					command_buffer_[4] = 0.0;
					std::cout << "LLLLLLLLLLLLLLLLLLLLLLLLLLLLL" << std::endl;
					break;
				}
				
				//command_buffer_[1] = std::min(std::max(0.0f, std::abs(vel)), 1.0f);
				//command_buffer_[2] = 0.0;
				//command_buffer_[3] = 0.0;
				//command_buffer_[4] = 0.0;
				if (send) {
					this->send_command();
					is_moving_ = true;
				}
			}
		}

		void Michelangelo::stop()
		{
			if (is_dumping_) {
				// Stop
				command_buffer_[1] = 0.0;
				command_buffer_[2] = 0.0;
				command_buffer_[3] = 0.0;
				command_buffer_[4] = 0.0;
				command_buffer_[5] = 0.0;
				command_buffer_[6] = 0.0;
				command_buffer_[7] = 0.0;
				command_buffer_[8] = 0.0;					
				this->send_command();
				is_moving_ = false;
			}
		}

		void Michelangelo::send_command()
		{
			uint8_t packet[PACKET_OUT_LENGTH];
			size_t packet_byte_length = sizeof(packet) / sizeof(uint8_t);

			// 'Control mode' command entry/byte 0
			*(packet + 0) = int(1.0);
			std::cout << command_buffer_[0] << " (" << +*(packet + 0) << ") ";
			// Velocity command entries/bytes 1-8
			for (int i(1); i < packet_byte_length; ++i) {
				*(packet + i) = int(command_buffer_[i] * 255);
				std::cout << command_buffer_[i] << " (" << +*(packet + i) << ") ";
			}
			std::cout << std::endl;

			this->send_packet(packet, packet_byte_length);
		}

		/**** PROTECTED MEMBER FUNCTIONS ****/

		int Michelangelo::receive_packet(uint8_t packet[])
		{
			int packet_byte_length = HandUDP::receive_packet(packet);
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
			while (true) {
				uint8_t packet[1024];
				int byte_length(this->receive_packet(packet));

				if (byte_length > 0) {
					if (!is_dumping_) { is_dumping_ = true; }

					mu_configstate_.lock();

					/* byte5: "Grasp Type" (int8), 0 for palmar and 1 for lateral */
					int grasp_type = +*(packet + 5);
					if (grasp_type == 0) { configstate_.grasp_type = GRASP::PALMAR; }
					else if (grasp_type == 1) { configstate_.grasp_type = GRASP::LATERAL; }

					/* ---- bugged ---- byte6: "Aperture" (int8), range [0,100]%
					 * byte0: "Main Drive" (uint8), main drive, range [0,100]%
					 * -> Palmar = [0.000,0.110]m
					 * -> Lateral = [0.000,0.070]m */
					//int grasp_size = +int8_t(*(packet + 6));
					int grasp_size = +uint8_t(*(packet + 0));
					float _x_(0.0);
					switch (configstate_.grasp_type) {
					case GRASP::PALMAR:						
						if (160.0 <= float(grasp_size) && float(grasp_size) <= 255.0) {
							// Correct Phase
							_x_ = (float(grasp_size) - 160.0) / (255.0 - 160.0);							
						} else if(0.0 <= float(grasp_size) && float(grasp_size) <= 90.0) {
							// Transition Phase (inverted vals)
							_x_ = (90.0 - float(grasp_size)) / (90.0 - 0.0);
						}
						// Linear							
						configstate_.grasp_size = _x_ * 0.110;
						// Polynomial 4th-order
						configstate_.grasp_size = (0.6524 * std::pow(_x_, 4) - 2.4145 * std::pow(_x_, 3) + 2.1337 * std::pow(_x_, 2) + 0.6547 * _x_ - 0.0187) * 0.110;

						break;
					case GRASP::LATERAL:
						if (0.0 <= float(grasp_size) && float(grasp_size) <= 90.0) {
							// Correct Phase (inverted vals)
							_x_ = (90.0 - float(grasp_size)) / (90.0 - 0.0); 
						} else if (160.0 <= float(grasp_size) && float(grasp_size) <= 255.0) {
							// Transition Phase
							_x_ = (float(grasp_size) - 160.0) / (255.0 - 160.0);
						}
						// Linear							
						configstate_.grasp_size = _x_ * 0.070;
						// Polynomial 4th-order
						configstate_.grasp_size = (2.4965 * std::pow(_x_, 4) - 6.4698 * std::pow(_x_, 3) + 4.3496 * std::pow(_x_, 2) + 0.6082 * _x_ + 0.001) * 0.070;

						break;
					}

					/* byte7: "Pronation/Supination" (int8), range [-100,100]% -> [-160,160]deg */
					int sup_pro_angle = +int8_t(*(packet + 7));
					configstate_.sup_pro_angle = float(sup_pro_angle / 100.0 * 160 * M_PI / 180);

					/* byte8: "Flexion/Extension" (int8), range [-100,100]% -> [-45,75]deg */
					int fle_ext_angle = +int8_t(*(packet + 8));
					configstate_.fle_ext_angle = fle_ext_angle; // :=0.0

					/* byte9: "Force" (int8), range [0, 100]%
					 * -> Gripping force in Opposition/Palmar Mode = 70 N (commercial) or 100 N (research)
					 * -> Gripping force in Lateral Mode = 60 N
					 * -> Gripping force in Neutral Mode = 15 N */
					int  grasp_force = +int8_t(*(packet + 9));
					switch (configstate_.grasp_type) {
					case GRASP::PALMAR:
						configstate_.grasp_force = grasp_force / 100 * 100;
						break;
					case GRASP::LATERAL:
						configstate_.grasp_force = grasp_force / 100 * 60;
						break;
					}

					/* byte16-31: 1th-8th EMG channel (uint16), range [0,856] */
					size_t j(0);
					for (int i(16); i < 31; i += 2) {
						uint8_t most_significant_byte = *(packet + i + 1);
						uint8_t least_significant_byte = *(packet + i);
						uint16_t val = (most_significant_byte << 8) | least_significant_byte; // Little-Endian
						configstate_.emg_channels[j] = float(+val)/856.0;
						++j;
					}

					mu_configstate_.unlock();
				}
			}
		}
	}
}