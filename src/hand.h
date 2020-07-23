#pragma once

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

		class Hand
		{
		public:
			Hand() = delete;
			Hand(bool right_hand);
			~Hand();

			bool isRightHand();

			/* Set a configuration state for the prosthesis. */
			virtual void setConfigState() {}

			/* Read the current configuration state of the prosthesis. */
			virtual std::vector<float> getConfigState() { return std::vector<float>(); }

			void setControlMode(CONTROL ctrl);
			void setGrasp(GRASP grasp);

			virtual float getWristFleExtAngle();
			virtual float getWristAbdAddAngle();
			virtual float getWristSupProAngle();
			virtual float getGraspSize();
			virtual float getGraspForce();

			virtual void flex(float vel) {}
			virtual void extend(float vel) {}

			virtual void abduct(float vel) {}
			virtual void adduct(float vel) {}

			virtual void pronate(float vel) {}
			virtual void supinate(float vel) {}
			
			virtual void open(float vel) {}
			virtual void close(float vel) {}
			
			virtual void stop() {}

		protected:
			struct ConfigState {
				CONTROL ctrl_type;
				GRASP grasp_type;
				float fle_ext_angle = 0.0; //[rad]
				float abd_add_angle = 0.0; //[rad]
				float sup_pro_angle = 0.0; //[rad]
				float grasp_size = 0.0; //[m]
				float grasp_force = 0.0; //[N]
			} configstate_;

			bool right_hand_;

			virtual void updateConfigState() { std::cout << "SHOULD NOT READ THIS!" << std::endl; }

			void rotate();

			virtual void grasp_palmar() {}
			virtual void grasp_lateral() {}
		};
	}
}

