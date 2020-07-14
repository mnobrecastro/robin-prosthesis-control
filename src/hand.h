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

			Hand(bool right_hand);
			~Hand();

			bool isRightHand();

			void updateConfigState();

			/* Set a configuration state for the prosthesis. */
			virtual void setConfigState();

			/* Read the current configuration state of the prosthesis. */
			virtual std::vector<float> getConfigState() {}

			void setControlMode(CONTROL ctrl);
			void setGrasp(GRASP grasp);

			virtual float getFlexionAngle() {}
			virtual float getExtensionAngle() {}
			virtual float getWristFleExtAngle() {}

			virtual float getAbductionAngle() {}
			virtual float getAdductionAngle() {}
			virtual float getWristAbdAddAngle() {}

			virtual float getSupinationAngle() {}
			virtual float getPronationAngle() {}
			virtual float getWristSupProAngle() {}

			virtual float getOpenSize() {}
			virtual float getCloseSize() {}
			virtual float getGraspSize() {}

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

