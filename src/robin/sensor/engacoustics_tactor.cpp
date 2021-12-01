#include "engacoustics_tactor.h"

namespace robin {

	EngAcousticsTactor::EngAcousticsTactor()
	{
		data_ = 0.0;
		std::cout << "A new EngAcousticsTactor was created" << std::endl;

		//Initialize the TDK
		CheckForError(InitializeTI());

		//discovers tactor controllers
		int DeviceCount = Discover(DEVICE_TYPE_SERIAL);
		if (DeviceCount > 0)
		{
			//found a Device - Connect to it
			//This example connects to the first device it finds.
			DeviceID_ = Connect(GetDiscoveredDeviceName(0), DEVICE_TYPE_SERIAL, NULL);
			CheckForError(DeviceID_);
		}
		else {
			if (DeviceCount == 0)
				std::cerr << "EngAcousticsTactor: Discover found 0 devices" << std::endl;
			else
				CheckForError(DeviceCount);
			//return 0;
		}

		thread_feedback_ = std::thread(&EngAcousticsTactor::runFeedback, this);
	}
	EngAcousticsTactor::~EngAcousticsTactor() 
	{
		CheckForError(Close(DeviceID_));

		CheckForError(ShutdownTI());
	}

	void EngAcousticsTactor::setSample(const float value)
	{
		mu_data_.lock();
		data_ = value;
		mu_data_.unlock();
	}


	void EngAcousticsTactor::runFeedback()
	{
		while (DeviceID_ >= 0)
		{
			//Pulse a Tactor for given time(ms), at given delay(ms)
			CheckForError(Pulse(DeviceID_, 1, 50, 0));
			//Pulse a Tactor for given time(ms), at given delay(ms)
			CheckForError(Pulse(DeviceID_, 2, 50, 50));
			//Pulse a Tactor for given time(ms), at given delay(ms)
			CheckForError(Pulse(DeviceID_, 3, 50, 150));
			//Pulse a Tactor for given time(ms), at given delay(ms)
			CheckForError(Pulse(DeviceID_, 4, 50, 200));
			Sleep(400);

			/*if (--loopTimes <= 0)
			{
				printf("Continue? (y\\n): ");

				if (toupper(getc(stdin)) != 'Y')
					break;

				// loop 5 more times
				loopTimes = 5;

				int ch = 0;
				while ((ch = getchar()) != '\n' && ch != EOF);
			}*/

			//Update the TactorInterface (Will return any internal errors)
			CheckForError(UpdateTI());
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