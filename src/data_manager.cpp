#include "data_manager.h"

namespace robin
{
	namespace data {
				
		DataManager::DataManager(std::string folderpath)
		{
			folderpath_ = folderpath;

			std::string code, trial;
			std::cout << "Insert code: ";
			std::cin >> code;
			std::cout << "Insert trial number: ";
			std::cin >> trial;

			filename_calib_ = "c" + code + "_calib.txt";
			filename_events_ = "c" + code + "_data_" + trial + ".txt";

			// Checks if FOLDERPATH exists
			if (!std::filesystem::exists(folderpath_)) {
				std::filesystem::create_directory(folderpath_);
			}
			// Checks if the new sub-folder exists
			folderpath_ += "/" + code + "/";
			if (!std::filesystem::exists(folderpath_)) {
				std::filesystem::create_directory(folderpath_);
			}

			// Checks if a calibration file exists


			// Starts the inner clock (i.e. timestamp)
			t0_ = std::chrono::high_resolution_clock::now();
			ta_ = t0_;
			t_ = t0_;
		}

		DataManager::DataManager()
			: DataManager(FOLDERPATH)
		{}

		int DataManager::saveCalibration(const CalibData& data)
		{			
			std::string m("");
			for (size_t i(0); i < data.basevals.size(); ++i) {
				m += std::to_string(data.basevals[i]) + " ";
				m += std::to_string(data.normvals[i]);
				this->writeToFile(filename_calib_, m);
			}
			return 0;
		}

		int DataManager::loadCalibration(CalibData& data)
		{
			// Checks if a calibration file exists
			if (std::filesystem::exists(folderpath_ + filename_calib_)) {
				std::string m, line;
				this->readFromFile(filename_calib_, m);
				
				std::istringstream str(m);
				while (std::getline(str, line)) {
					std::string::size_type size;
					float baseval, normval;
					baseval = std::stof(line, &size);
					normval = std::stof(line.substr(size));
					data.basevals.push_back(baseval);
					data.normvals.push_back(normval);
					std::cout << baseval << " " << normval << std::endl;
				}
				return 0;
			}
			else {
				return 1;
			}
		}

		int DataManager::saveEvent(EventData& data)
		{
			// Update timetags before saving
			auto t = std::chrono::high_resolution_clock::now();
			//std::chrono::duration<double, std::ratio<1>> duration = t - t_;
			data.t_epoch = std::chrono::duration<double, std::ratio<1>>(t - t0_).count(); // Absolute time (begining of the collection)
			data.t_attempt = std::chrono::duration<double, std::ratio<1>>(t - ta_).count(); // Time from the beginning of the attemp
			data.t_event = std::chrono::duration<double, std::ratio<1>>(t - t_).count(); // Time from the last event
			if (data.flag) {
				ta_ = t;
				data.flag = false;
			}
			t_ = t;

			std::string m("");
			m += std::to_string(data.t_epoch) + " "; // Absolute time (begining of the collection)
			m += std::to_string(data.t_attempt) + " "; // Time from the attemp
			m += std::to_string(data.t_event) + " "; // Time between events
			m += std::to_string(data.mode) + " "; // Mode Auto_1/Manual_0
			m += std::to_string(data.rotate) + " "; // Rotation
			m += std::to_string(data.grasp_type) + " ";
			m += std::to_string(data.grasp_size_estim) + " "; // grasp_size from the CVsolver
			m += std::to_string(data.grasp_size_slack) + " "; // grasp_size with slack/tolerance
			m += std::to_string(data.tilt_angle_estim) + " "; // tilt_angle from the CVsolver
			m += std::to_string(data.tilt_angle_slack); // tilt_angle with slack/tolerance
			
			return this->writeToFile(filename_events_, m);
		}


		int DataManager::writeToFile(std::string filename, const std::string& message)
		{
			std::ofstream f;
			f.open(folderpath_ + filename, std::fstream::app);
			if (f.is_open()){
				f << message + "\n";
				f.close();
				return 0;
			}
			else {
				return 1;
			}
			
		}

		int DataManager::readFromFile(std::string filename, std::string& message)
		{
			std::string line;
			std::ifstream f;
			f.open(folderpath_ + filename);
			if (f.is_open()) {
				while (std::getline(f, line)) {
					message += line + "\n";
				}
				f.close();
				return 0;
			}
			else {
				return 1;
			}			
		}
	}
}