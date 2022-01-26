#include "solver1.h"

namespace robin
{
	Solver1::Solver1(size_t N)
	{
		BUFFER_LEN_ = N;
		for (int i(0); i < N; ++i) {
			raw_buffer_.push_back(0.0);
			data_buffer_.push_back(0.0);
		}
	}

	Solver1::~Solver1() {}

	float Solver1::getSample()
	{
		float val(0.0);
		mu_data_.lock();
		val = data_buffer_.back();
		mu_data_.unlock();
		return val;
	}

	float Solver1::getMean(size_t w)
	{
		float val(0.0f);
		for (auto sample : this->getWindow(w)) {
			val += sample;
		}
		return val/w;
	}

	std::vector<float> Solver1::getWindow(size_t w)
	{
		std::vector<float> vec;
		mu_data_.lock();
		vec = std::vector<float>(data_buffer_.end()-w, data_buffer_.end());
		mu_data_.unlock();
		return vec;
	}

	std::vector<float> Solver1::getData()
	{
		std::vector<float> vec;
		mu_data_.lock();
		vec = std::vector<float>(data_buffer_);
		mu_data_.unlock();
		return vec;
	}

	std::vector<float> Solver1::getPreprocessed()
	{
		std::vector<float> vec;
		mu_raw_.lock();
		vec = std::vector<float>(raw_buffer_);
		mu_raw_.unlock();
		return vec;
	}

	void Solver1::addSensor(robin::Sensor1* sensor)
	{
		sensors_.push_back(sensor);
	}

	void Solver1::setFilter(fname f, std::size_t window_size)
	{
		filterOnOff_ = true;
		filt_ = f;
		window_size_ = window_size;
	}

	void Solver1::setBaselineRemoval(float val)
	{
		baselineOnOff_ = true;
		baseval_ = val;
	}

	void Solver1::setNormalization(float val, bool saturate)
	{
		normalizeOnOff_ = true;
		saturateOnOff_ = saturate;
		normval_ = val;
	}



	void Solver1::solve()
	{		
		// Retrieve the most recent sample from the sensor
		this->readSample();

		float val(0.0);

		this->filter(val, filt_, window_size_);

		this->baseline(val);

		this->normalize(val);

		// Update the buffer/signal according to the chosen filter
		this->setSample(val);
	}

	void Solver1::readSample()
	{
		float val(0.0);
		if (sensors_.size() == 1) {
			val = sensors_[0]->getSample();
		}
		mu_raw_.lock();
		raw_buffer_.push_back(val);
		raw_buffer_.erase(raw_buffer_.begin()); /* Pseudo FIFO (TO DO: deque for larger buffers) */
		mu_raw_.unlock();
	}

	void Solver1::filter(float& val, fname f, std::size_t window_size)
	{
		switch (f) {
		case fname::NONE:
			val = this->f_None();
			break;
		case fname::MEDIAN:
			val = this->f_Median(window_size);
			break;
		case fname::MODE:
			val = this->f_Mode(window_size);
			break;
		case fname::MOVING_AVERAGE:
			val = this->f_MovingAverage(window_size);
			break;
		case fname::EXP_MOVING_AVERAGE:
			//val = this->f_ExpMovingAverage(window_size);
			break;
		}
	}

	void Solver1::baseline(float& val)
	{
		if (baselineOnOff_) {
			val -= baseval_;
		}
	}

	void Solver1::normalize(float& val)
	{
		if (normalizeOnOff_) {
			val /= normval_;
			if (saturateOnOff_ && val > 1.0) {
				val = 1.0;
			}
		}
	}

	/* Median Value window filter */
	float Solver1::f_None()
	{
		// Retrive the last value in the raw_buffer
		float val(0.0);
		mu_raw_.lock();
		val = raw_buffer_.back();
		mu_raw_.unlock();		
		return val;
	}

	/* Median Value window filter */
	float Solver1::f_Median(std::size_t window_size)
	{
		// Retrive the window of a given size from the raw_buffer
		std::vector<float> temp;
		mu_raw_.lock();
		if (raw_buffer_.size() < window_size) {
			temp = std::vector<float>(raw_buffer_.begin(), raw_buffer_.end());
		} else {
			temp = std::vector<float>(raw_buffer_.end() - window_size, raw_buffer_.end());
		}
		mu_raw_.unlock();

		// Calculate the new filtered value and push it to the data_buffer
		float val(0.0);
		std::sort(temp.begin(), temp.end());
		if (temp.size() % 2 == 0) {
			val = (temp[temp.size() / 2 - 1] + temp[temp.size() / 2]) / 2.0;
		} else {
			val = temp[temp.size() / 2];
		}
		return val;
	}

	/* Mode (Majority Voting) window filter */
	float Solver1::f_Mode(std::size_t window_size)
	{
		// Retrive the window of a given size from the raw_buffer
		std::vector<float> temp;
		mu_raw_.lock();
		if (raw_buffer_.size() < window_size) {
			temp = std::vector<float>(raw_buffer_.begin(), raw_buffer_.end());
		} else {
			temp = std::vector<float>(raw_buffer_.end() - window_size, raw_buffer_.end());
		}
		mu_raw_.unlock();

		// Calculate the new filtered value and push it to the data_buffer
		float val(0.0);
		std::sort(temp.begin(), temp.end());
		float cur_val(temp[0]), best_val(temp[0]);
		size_t cur_count(1), best_count(1);
		for (size_t i(1); i < temp.size(); ++i) {
			if (temp[i] == cur_val) {
				++cur_count;
				if (cur_count > best_count) {
					best_count = cur_count;
					best_val = cur_val;
				}
			}
			else {
				cur_count = 1;
				cur_val = temp[i];
			}
		}
		val = best_val;
		return val;
	}

	/* Moving Average window filter */
	float Solver1::f_MovingAverage(std::size_t window_size)
	{
		std::vector<float> temp;
		mu_raw_.lock();
		if (raw_buffer_.size() < window_size) {
			temp = std::vector<float>(raw_buffer_.begin(), raw_buffer_.end());
		}
		else {
			temp = std::vector<float>(raw_buffer_.end() - window_size, raw_buffer_.end());
		}
		mu_raw_.unlock();

		// Calculate the new filtered value and push it to the data_buffer
		float val(0.0);
		for (auto a : temp) {
			val += a;
		}
		val /= temp.size();
		return val;
	}

	void Solver1::setSample(float& val)
	{
		// Copy the value to the data_buffer
		mu_data_.lock();
		data_buffer_.push_back(val);
		data_buffer_.erase(data_buffer_.begin()); /* Pseudo FIFO (TO DO: deque for larger buffers) */
		mu_data_.unlock();
	}
}