#pragma once
#include "hand.h"
#include "solver.h"

#include <vector>

namespace robin
{
	namespace control
	{
		struct ControlVar {
			float value;
			std::vector<float> buffer;
		};

		class Control
		{
		public:
			Control() {};
			void addControlVar(float* var);

		protected:
			robin::hand::Hand* hand_;
			std::vector<ControlVar> ctrl_vars_;
		};

	}
}