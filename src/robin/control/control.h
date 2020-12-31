#pragma once
#include "robin/sensor/hand.h"
#include "robin/solver/solver.h"

#include <vector>

namespace robin
{
	namespace control
	{
		struct ControlVar {
			float value;
			std::vector<float> buffer;

			enum fname {
				MEDIAN,
				MODE,
				MOVING_AVERAGE,
				EXP_MOVING_AVERAGE
			};

			//void update(void (&func) (size_t), size_t window_size)
			void update(fname f, size_t window_size)
			{
				switch (f) {
				case fname::MEDIAN:
					this->f_Median(window_size);
					break;
				case fname::MODE:
					this->f_Mode(window_size);
					break;
				case fname::MOVING_AVERAGE:
					this->f_MovingAverage(window_size);
					break;
				case fname::EXP_MOVING_AVERAGE:
					//this->f_ExpMovingAverage(window_size);
					break;
				}
				
			}

		private:

			/* Median Value window filter */
			void f_Median(size_t window_size)
			{
				std::vector<float> temp;
				if (this->buffer.size() < window_size) {
					temp = this->buffer;
				} else {
					temp = std::vector<float>(this->buffer.end() - window_size, this->buffer.end());
				}

				std::sort(temp.begin(), temp.end()); // , temp.begin()
				if (temp.size() % 2 == 0) {
					this->value = (temp[temp.size()/2 - 1] + temp[temp.size()/2]) / 2.0; //(int)temp.size
				} else {
					this->value = temp[temp.size()/2];
				}
			}
			
			/* Mode (Majority Voting) window filter */
			void f_Mode(size_t window_size) {
				std::vector<float> temp;
				if (this->buffer.size() < window_size) {
					temp = this->buffer;
				}
				else {
					temp = std::vector<float>(this->buffer.end() - window_size, this->buffer.end());
				}

				std::sort(temp.begin(), temp.end()); // , temp.begin()
				float cur_val(temp[0]), best_val(temp[0]);
				size_t cur_count(1), best_count(1);
				for (int i(1); i < temp.size(); ++i) {
					if (temp[i] == cur_val) {
						++cur_count;
						if (cur_count > best_count) {
							best_count = cur_count;
							best_val = cur_val;
						}
					} else {						
						cur_count = 1;
						cur_val = temp[i];
					}
				}
				this->value = best_val;
			}

			/* Moving Average window filter */
			void f_MovingAverage(size_t window_size)
			{
				std::vector<float> temp;
				if (this->buffer.size() < window_size) {
					temp = this->buffer;
				}
				else {
					temp = std::vector<float>(this->buffer.end() - window_size, this->buffer.end());
				}

				this->value = 0.0;
				for (auto a : temp) {
					this->value += a;
				}
				this->value /= temp.size();
			}
		};

		class Control
		{
		public:
			Control() {};
			void addControlVar(float* var);

			void setFilter(ControlVar::fname f, size_t n) {
				filter_ = f;
				window_size_ = n;
			}

		protected:
			robin::hand::Hand* hand_;
			std::vector<ControlVar> ctrl_vars_;

			size_t window_size_ = 1;
			ControlVar::fname filter_ = ControlVar::fname::MEDIAN;
		};

	}
}