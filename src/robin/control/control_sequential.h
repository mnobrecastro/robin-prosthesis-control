#pragma once
#include "control.h"
#include "robin/primitive/primitive3.h"
#include "robin/primitive/primitive3_sphere.h"
#include "robin/primitive/primitive3_cuboid.h"
#include "robin/primitive/primitive3_cylinder.h"
// ---- dynamic_cast
#include "robin/sensor/hand_michelangelo.h"
// ----
#include "robin/utils/data_manager.h"

#include <typeinfo>
#include <algorithm>
#include <cmath>

#include <chrono>
#include <thread>

#include <windows.h>
#include <conio.h>

namespace robin
{
	namespace control
	{
		enum class Primitive3Type {
			PRIMITIVE3_CUBOID,
			PRIMITIVE3_CYLINDER,
			PRIMITIVE3_SPHERE,
			UNKNOWN
		};

		enum class MyocontrolMenu {
			MYOMENU_PALMAR,
			MYOMENU_LATERAL,
			MYOMENU_ROTATE
		};

		class ControlSequential :
			public Control
		{
		public:
			ControlSequential(robin::hand::Hand& hand);

			float getGraspSize();
			float getTiltAngle();
			std::vector<float> getEMG();

			bool getStateAuto() { return state_auto_; }
			bool getStateGrasp() { return state_grasp_; }

			void setFullManual(bool b=false) { full_manual_ = b; }

			void setDataManager(robin::data::DataManager& dm);

			/* Evaluates the controller at a new time instant. */
			void evaluate(robin::Primitive3* prim);

		protected:
			ControlVar emg_cmd_flexion_;
			ControlVar emg_cmd_extension_;
			ControlVar force_detection_;

			// Channel locking variables
			bool emg0_lock_ = false;
			std::chrono::steady_clock::time_point emg0_time_;
			bool emg1_lock_ = false;
			std::chrono::steady_clock::time_point emg1_time_;

			// Full manual control option
			bool full_manual_ = false;
			// State auto:=true corresponds to "auto" mode, while move:=false corresponds to "manual" mode
			bool state_auto_ = true;
			bool flag_switch_ = false;
			// State rotation:=true corresponds to pro/sup movements, rotation:=false corresponds to 
			MyocontrolMenu state_myomenu_ = MyocontrolMenu::MYOMENU_PALMAR;
			bool flag_coactiv_ = false;
			// State grasp:=false corresponds to grasp "force" detected
			bool state_grasp_ = false;

			ControlVar hand_supination_angle_;
			ControlVar hand_grasp_size_;

			ControlVar target_grasp_size_;
			ControlVar target_tilt_angle_;
			ControlVar target_grasp_type_;

			bool use_saved_targets_ = false;
			float saved_grasp_size_;
			float saved_tilt_angle_;
			int saved_grasp_type_;
			float last_grasp_size_ = 0.1;
			float last_tilt_angle_ = 0.0;
			int last_grasp_type_ = int(robin::hand::GRASP::PALMAR);

			/* Receives a Primitive3 object and evaluates its type. */
			Primitive3Type find_primitive3_type(robin::Primitive3* prim);

			/* Estimation of the "grasp size" from a primitive3 object. */
			void estimate_grasp_size(robin::Primitive3* prim);

			/* Estimation of the "grasp type" from a primitive3 object. */
			void estimate_grasp_type(robin::Primitive3* prim);

			/* Estimation of the "tilt angle" from a primitive3 object. */
			void estimate_tilt_angle(robin::Primitive3* prim);						



			/* Declaration of a DataManager object. */
			robin::data::DataManager* dm_ = nullptr;
			int saveEvent(bool flag=false) const;

			/* isKeyPressed aux method to reset the hand position. */
			// (NOTE: WindowsOS only! )
			char key_stored_;
			bool key_pressed_ = false;
			bool flag_key_ = false;
			bool isKeyPressed(char* c);
		};
	}
}