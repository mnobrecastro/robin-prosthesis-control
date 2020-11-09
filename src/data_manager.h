#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <vector>
#include <chrono>

#define FOLDERPATH "../data"

namespace robin
{
	namespace data
	{
		struct CalibData {
			std::vector<float> basevals;
			std::vector<float> normvals;
		};

		struct EventData {
			float t_epoch; // Absolute time (begining of the collection)
			float t_attempt; // Time from the attemp
			float t_event; // Time between events
			int mode; // Mode Auto_0/Manual_1
			int rotate; // Rotation
			int grasp_type;
			float grasp_size_estim; // grasp_size from the CVsolver
			float grasp_size_slack; // grasp_size with slack/tolerance
			float tilt_angle_estim; // tilt_angle from the CVsolver
			float tilt_angle_slack; // tilt_angle with slack/tolerance
			bool flag = false;
		};
		
		class DataManager
		{
		public:
			DataManager();
			DataManager(std::string s);
			~DataManager() {}
			
			int saveCalibration(const CalibData& data);

			int loadCalibration(CalibData& data);

			int saveEvent(EventData& data);

		protected:
			std::string folderpath_;
			std::string filename_calib_;
			std::string filename_events_;

			std::chrono::steady_clock::time_point t0_, ta_, t_;

			int writeToFile(std::string filename, const std::string& s);
			int readFromFile(std::string filename, std::string& s);
		};

	}
}