#pragma once
#include "tactor.h"

#include <iostream>
#include <vector>
#include <array>

#ifdef _WIN32
#include <Windows.h>
#include <TactorInterface.h>
#pragma comment(lib, "TactorInterface.lib")
#else
#include <cstdio>
#include <TactorInterface.h>
#define Sleep(x) usleep((x)*1000);
#endif // _WIN32

namespace robin
{
	class EngAcousticsTactor :
		public Tactor
	{
	public:
		EngAcousticsTactor() = delete;
		EngAcousticsTactor(const std::vector<int> channels, float gainfactor=1.0, char* port=" ");
		~EngAcousticsTactor();

		//void setSample(std::array<float,2> vals);

	protected:
		//float getSample() = delete;
		//std::array<float,2> getSample();

		void updateTactor();

		void update();

	private:
		int DeviceID_ = -1;
		float gainFactor_;

		// Channels
		std::vector<int> ch_idx_ = {};
		std::array<float, 4> ch_val_ = { 0.0, 0.0, 0.0, 0.0 };

		bool mode_idle_ = true;
		bool mode_centered_ = false;

		void CheckForError(int errorCode);
	};

}