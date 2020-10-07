#pragma once
#include "solver1.h"

namespace robin {

	class Solver1EMG :
		public Solver1
	{
	public:
		Solver1EMG() {};
		~Solver1EMG() {};

		void calibrate();

		//void solve();

	protected:

		bool is_calibrated_ = false;

		
	};
}