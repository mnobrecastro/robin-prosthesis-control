#pragma once
#include "sensor1.h"
#include "solver1_emg.h"
//#include "data_manager.h"

#include <iostream>
#include <vector>

namespace robin
{
	namespace hand
	{
		enum class CONTROL {			
			POSITION,
			VELOCITY,
			FORCE
		};

		enum class GRASP {
			PALMAR,
			LATERAL
		};

		class Hand :
			public Sensor1
		{
		public:
			Hand() = delete;
			Hand(bool right_hand);
			~Hand();

			bool isRightHand();

			/* Set a configuration state for the prosthesis. */
			virtual void setConfigState() {}

			void setControlMode(CONTROL ctrl);
			void setGrasp(GRASP grasp);

			/* Read the current configuration state of the prosthesis. */
			virtual std::vector<float> getConfigState() { return std::vector<float>(); }

			/* Read the current sEMG channels provided by the prosthesis. */
			virtual std::vector<float> getEMG() { return std::vector<float>(); }//{ return configstate_.emg_channels; }

			virtual std::vector<Solver1EMG*> getEMGSolvers() { std::vector<Solver1EMG*>(); }

			virtual float getWristFleExtAngle();
			virtual float getWristAbdAddAngle();
			virtual float getWristSupProAngle();
			virtual float getGraspSize();
			virtual float getGraspForce();			

			virtual void flex(float vel, bool send = true) {}
			virtual void extend(float vel, bool send = true) {}

			virtual void abduct(float vel, bool send = true) {}
			virtual void adduct(float vel, bool send = true) {}

			virtual void pronate(float vel, bool send = true) {}
			virtual void supinate(float vel, bool send = true) {}
			
			virtual void open(float vel, bool send = true) {}
			virtual void close(robin::hand::GRASP g, float vel, bool send = true) {}
			
			virtual void stop() {}

			virtual void send_command() {}

			//virtual void setDataManager(robin::data::DataManager& dm) {}

		protected:
			struct ConfigState {
				CONTROL ctrl_type;
				GRASP grasp_type;
				float fle_ext_angle = 0.0; //[rad]
				float abd_add_angle = 0.0; //[rad]
				float sup_pro_angle = 0.0; //[rad]
				float grasp_size = 0.0; //[m]
				float grasp_force = 0.0; //[N]
				std::vector<float> emg_channels; //[0.0-1.0]
			} configstate_;

			bool right_hand_;

			virtual void updateConfigState() { std::cout << "SHOULD NOT READ THIS!" << std::endl; }

			void rotate();

			virtual void grasp_palmar() {}
			virtual void grasp_lateral() {}
		};
	}
}

