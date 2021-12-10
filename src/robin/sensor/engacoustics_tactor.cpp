#include "engacoustics_tactor.h"

namespace robin {

	EngAcousticsTactor::EngAcousticsTactor(const std::vector<int> channels, float gainfactor, char* port)
	{
		try {
			if (channels.size() == 4) {
				// Verify the current number and range of allowed channels
				bool bAllowed(true);
				bool bRepeated(false);
				std::vector<int> chs;
				for (auto ch : channels) {
					// Within range
					if (ch < 1 || ch > 8) { 
						bAllowed = false;
						break;
					}
					// Repeated channels
					for (auto c : chs){
						if (ch == c) {
							bRepeated = true;
							break;
						}
						
					}
					chs.push_back(ch);
				}
				if (bAllowed && !bRepeated) {
					ch_idx_ = channels;
				}
				else {
					if (!bAllowed)
						throw std::out_of_range("EngAcousticsTactor: One or more channels out-of-range [1-8].");
					if (bRepeated)
						throw std::invalid_argument("EngAcousticsTactor: One or more channels repeated.");
				}
			}
			else
				throw std::invalid_argument("EngAcousticsTactor: Invalid channels size (4 required).");
			
			if(gainfactor <= 0.0 || gainfactor > 1.0)
				throw std::out_of_range("EngAcousticsTactor: gainFactor out-of-range ]0.0,1.0].");
		}
		catch (std::logic_error& e){
			std::cout << e.what() << '\n';
			throw;
		}

		// Initialize the TDK Interface
		CheckForError(InitializeTI());

		if (port != " ") {
			// Connect to a Tactor Controller Unit
			printf("Connecting to the Tactor Controller Unit... (Port: %s)\n", port);				
			DeviceID_ = Connect(port, DEVICE_TYPE_SERIAL, NULL);
			if (DeviceID_ < 0) {
				CheckForError(DeviceID_);
				printf("Error while connecting the TCU at Port: %s\n", port);
			}
		}
		if(DeviceID_ < 0){
			// Discover Tactor Controller Units
			int DeviceCount = Discover(DEVICE_TYPE_SERIAL);
			if (DeviceCount > 0) {
				// Connect to the first device found
				std::cout << "Discovering a Tactor Controller Unit device...\n";
				DeviceID_ = Connect(GetDiscoveredDeviceName(0), DEVICE_TYPE_SERIAL, NULL);
				if (DeviceID_ < 0) {
					CheckForError(DeviceID_);
					std::cout << "Error while connecting to the TCU device.\n";
				}
			}
			else {
				// No device was discovered
				if (DeviceCount == 0)
					std::cerr << "EngAcousticsTactor: 0 devices found.\n";
				else {
					CheckForError(DeviceCount);
					std::cout << "Error while discovering a TCU device.\n";
				}
			}			
		}

		gainFactor_ = gainfactor;

		std::cout << "A new EngAcousticsTactor was created\n";

		thread_feedback_ = std::thread(&EngAcousticsTactor::updateFeedback, this);
	}
	
	EngAcousticsTactor::~EngAcousticsTactor()
	{
		CheckForError(Close(DeviceID_));
		CheckForError(ShutdownTI());
		thread_feedback_.join();
	}


	void EngAcousticsTactor::update()
	{
		std::array<float,2> data = this->getSample();
		float rho, theta;
		// Data truncation
		//// rho
		if (data[0] > 1.0)
			rho = 1.0;
		else
			rho = data[0];
		//// theta
		if (data[1] < -M_PI)
			theta = -M_PI;
		else if (data[1] > M_PI)
			theta = M_PI;
		else
			theta = data[1];


		// Set FLAGs
		if (rho == -1.0) {
			// No PointCloud detected
			mode_idle_ = true;
		}
		else{
			mode_idle_ = false;
			if (rho > 0.0) {
				// PointCloud being detected
				mode_trigger_ = false;
			}
			else {
				// PointCloud is centered
				mode_trigger_ = true;
			}
		}

		// Set channel values
		if (theta >= 0.0 && theta < M_PI/2.0) {
			ch_val_[0] = (1.0 - rho) * (1.0 - theta / (M_PI / 2.0));
			ch_val_[1] = (1.0 - rho) * theta / (M_PI / 2.0);
			ch_val_[2] = 0.0;
			ch_val_[3] = 0.0;
		}
		else if (theta >= M_PI/2.0 && theta < M_PI) {
			ch_val_[0] = 0.0;
			ch_val_[1] = (1.0 - rho) * (1.0 - (theta-M_PI/2.0) / (M_PI / 2.0));
			ch_val_[2] = (1.0 - rho) * (theta-M_PI/2.0) / (M_PI / 2.0);
			ch_val_[3] = 0.0;
		}
		else if (theta >= -M_PI && theta < -M_PI/2.0) {
			ch_val_[0] = 0.0;
			ch_val_[1] = 0.0;
			ch_val_[2] = (1.0 - rho) * (theta+M_PI/2.0) / (-M_PI / 2.0);
			ch_val_[3] = (1.0 - rho) * (1.0 - (theta+M_PI/2.0) / (-M_PI / 2.0));
		}
		else {
			// theta >= -M_PI/2.0 && theta < 0.0
			ch_val_[0] = (1.0 - rho) * (1.0 - theta / (-M_PI / 2.0));
			ch_val_[1] = 0.0;
			ch_val_[2] = 0.0;
			ch_val_[3] = (1.0 - rho) * theta / (-M_PI / 2.0);
		}

		return;
	}

	void EngAcousticsTactor::updateFeedback()
	{		
		int tdiv, delayTime; // range 1-255: delay * timefactor -> on time 255 * 10
		int timeFactor(1000); // ms
		
		while (DeviceID_ >= 0) {

			this->update();
			
			if (mode_idle_) {
				// No PointCloud detected				
				Pulse(DeviceID_, ch_idx_[0], 0, 0);
				Pulse(DeviceID_, ch_idx_[1], 0, 0);
				Pulse(DeviceID_, ch_idx_[2], 0, 0);
				Pulse(DeviceID_, ch_idx_[3], 0, 0);
				Sleep(10);
			}
			else {				
			
				if(!mode_trigger_){
					// PointCloud being detected
					int ch_val_adj[4] = {
						ch_val_[0] * gainFactor_ * 255,
						ch_val_[1] * gainFactor_ * 255,
						ch_val_[2] * gainFactor_ * 255,
						ch_val_[3] * gainFactor_ * 255
					};

					/* TDK throws an error and vibrates a tactor if a ChangeGain is set to 0 [0-255]
					 * while Pulse is also declared. To overcome this issue a max() function is used
					 * to ensure the minimal gain is set to 1, which would not result in ant motor
					 * vibration. */
					ChangeGain(DeviceID_, ch_idx_[0], max(ch_val_adj[0], 1), 0);
					ChangeGain(DeviceID_, ch_idx_[1], max(ch_val_adj[1], 1), 0);
					ChangeGain(DeviceID_, ch_idx_[2], max(ch_val_adj[2], 1), 0);
					ChangeGain(DeviceID_, ch_idx_[3], max(ch_val_adj[3], 1), 0);
					Pulse(DeviceID_, ch_idx_[0], timeFactor, 0);
					Pulse(DeviceID_, ch_idx_[1], timeFactor, 0);
					Pulse(DeviceID_, ch_idx_[2], timeFactor, 0);
					Pulse(DeviceID_, ch_idx_[3], timeFactor, 0);
					Sleep(10); // ms
					std::cout << "********************************** " << int(ch_val_[0] * gainFactor_ * 255) << " " << int(ch_val_[1] * gainFactor_ * 255) << " " << int(ch_val_[2] * gainFactor_ * 255) << " " << int(ch_val_[3] * gainFactor_ * 255);
					std::cout << " | " << min(int(ch_val_[0] * 255), 1) << " " << min(int(ch_val_[1] * 255), 1) << " " << min(int(ch_val_[2] * 255), 1) << " " << min(int(ch_val_[3] * 255), 1) << '\n';
				}
				else {
					// PointCloud is centered
					//int timeFactor(100); // 10 = 10ms
					//SetTimeFactor(timeFactor);
					tdiv = 2;
					delayTime = 100 / tdiv;
					
					ChangeGain(DeviceID_, ch_idx_[0], 0.5 * gainFactor_ * 255 + 0.5f, 0);
					ChangeGain(DeviceID_, ch_idx_[1], 0.5 * gainFactor_ * 255 + 0.5f, 0);
					ChangeGain(DeviceID_, ch_idx_[2], 0.5 * gainFactor_ * 255 + 0.5f, 0);
					ChangeGain(DeviceID_, ch_idx_[3], 0.5 * gainFactor_ * 255 + 0.5f, 0);
					Pulse(DeviceID_, ch_idx_[0], 10, 0);
					Pulse(DeviceID_, ch_idx_[1], 10, 0);
					Pulse(DeviceID_, ch_idx_[2], 10, 0);
					Pulse(DeviceID_, ch_idx_[3], 10, 0);
					Pulse(DeviceID_, ch_idx_[0], 0, 10);
					Pulse(DeviceID_, ch_idx_[1], 0, 10);
					Pulse(DeviceID_, ch_idx_[2], 0, 10);
					Pulse(DeviceID_, ch_idx_[3], 0, 10);
					Sleep(20);
				}
			}

			// Update the TactorInterface (Will return any internal errors)
			UpdateTI();
		}
	}

	void EngAcousticsTactor::CheckForError(int errorCode)
	{
		//check if something went wrong
		if (errorCode < 0)
		{
			//gets the last error code recorded in the tactorinterface
			std::cout << "Error Code " << GetLastEAIError() << "\nCheck EAI_Defines.h For Reason\n";
		}
	}
}