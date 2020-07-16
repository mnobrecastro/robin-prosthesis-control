#pragma once

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

			void updateConfigState();

			/* Set a configuration state for the prosthesis. */
			virtual void setConfigState() {}

			/* Read the current configuration state of the prosthesis. */
			virtual std::vector<float> getConfigState() { return std::vector<float>(); }

			void setControlMode(CONTROL ctrl);
			void setGrasp(GRASP grasp);

			virtual float getFlexionAngle() { return 0.0; }
			virtual float getExtensionAngle() { return 0.0; }
			virtual float getWristFleExtAngle() { return 0.0; }

			virtual float getAbductionAngle() { return 0.0; }
			virtual float getAdductionAngle() { return 0.0; }
			virtual float getWristAbdAddAngle() { return 0.0; }

			virtual float getSupinationAngle() { return 0.0; }
			virtual float getPronationAngle() { return 0.0; }
			virtual float getWristSupProAngle() { return 0.0; }

			virtual float getOpenSize() { return 0.0; }
			virtual float getCloseSize() { return 0.0; }
			virtual float getGraspSize() { return 0.0; }

			virtual void flex() {}
			virtual void extend() {}

			virtual void abduct() {}
			virtual void adduct() {}

			virtual void pronate() {}
			virtual void supinate() {}
			
			virtual void open() {}
			virtual void close() {}
			

		protected:
			struct ConfigState {
				CONTROL ctrl;
				GRASP grasp;
			} configstate_;

			bool right_hand_;

			void rotate();

			virtual void grasp_palmar() {}
			virtual void grasp_lateral() {}
		};
	}
}

