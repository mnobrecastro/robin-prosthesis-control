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

			Hand();
			~Hand();

			void setControlMode(CONTROL ctrl);
			void setGrasp(GRASP grasp);

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

			void rotate();

			virtual void grasp_palmar() {}
			virtual void grasp_lateral() {}
		};
	}
}

