#pragma once
#include "tactor.h"

#include <iostream>

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
		EngAcousticsTactor();
		~EngAcousticsTactor();

		void setSample(float value);

	protected:
		float getSample() = delete;

		void runFeedback();

	private:
		int DeviceID_ = -1;

		void CheckForError(int errorCode);
	};

}