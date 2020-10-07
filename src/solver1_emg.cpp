#include "solver1_emg.h"

namespace robin
{
	void Solver1EMG::calibrate()
	{
		is_calibrated_ = true;

		data_buffer_.clear();
		raw_buffer_.clear();
	}
}