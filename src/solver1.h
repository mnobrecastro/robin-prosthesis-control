#pragma once
#include "solver.h"
#include "sensor1.h"

#include <vector>
#include <algorithm>

namespace robin
{
	/* Filtering functions available to process raw data. */
	enum fname {
		NONE,
		MEDIAN,
		MODE,
		MOVING_AVERAGE,
		EXP_MOVING_AVERAGE
	};

	class Solver1 :
		public Solver
	{
	public:
		Solver1();
		~Solver1();

		float getSample();

		std::vector<float> getData();

		std::vector<float> getPreprocessed();

		void addSensor(robin::Sensor1*);

		//void setSegmentation(pcl::SACSegmentation<pcl::PointXYZ>* seg_obj);

		//void setUseNormals(bool seg_normals);

		//void setPlaneRemoval(bool seg_plane_removal);

		//void setCrop(float, float, float, float, float, float);

		//void setDownsample(float);

		//void setResample(size_t order, float radius);

		//void setFairSelection(bool fairness);

		//std::vector<robin::Sensor3*> getSensors() const;

		void setFilter(fname f, std::size_t window_size);

		void solve();

		//virtual void visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const;

	protected:

		std::vector<robin::Sensor1*> sensors_;

		/* Float data (temp) that can be manipulated. */
		std::vector<float> data_buffer_;
		std::mutex mu_data_;

		/* Raw data. */
		std::vector<float> raw_buffer_;
		std::mutex mu_raw_;

		/* Filtering vars. */
		bool FILTER_ = false;
		fname filt_ = fname::NONE;
		std::size_t window_size_ = 1;


		/* No filter */
		void f_None();
		/* Median Value window filter */
		void f_Median(std::size_t window_size);
		/* Mode (Majority Voting) window filter */
		void f_Mode(std::size_t window_size);
		/* Moving Average window filter */
		void f_MovingAverage(std::size_t window_size);
		/* Exponential Moving Average window filter */
		void f_ExpMovingAverage(std::size_t window_size);

		/* The update method works, on purpose, at a different pace (F) of that of the sensor (Fs).
		 * Hence, some new samples constantly made available by the sensor are dropped in case the
		 * processing time of the Solver1::update() works at F < Fs. */
		void update(fname f, std::size_t window_size);
	};
}