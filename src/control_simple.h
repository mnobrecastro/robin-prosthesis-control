#pragma once
#include "control.h"
#include "primitive3.h"
#include "primitive3_sphere.h"
#include "primitive3_cuboid.h"
#include "primitive3_cylinder.h"
// ---- dynamic_cast
#include "hand_michelangelo.h"
// ----

#include <typeinfo>
#include <algorithm>
#include <cmath>

#include <chrono>
#include <thread>

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

		class ControlSimple :
			public Control
		{
		public:
			ControlSimple(robin::hand::Hand& hand);

			/* Evaluates the controller at a new time instant. */
			void evaluate(robin::Primitive3* prim);

			float getGraspSize();
			float getTiltAngle();
			std::vector<float> getEMG();

			bool getStateAuto() { return state_auto_; }
			bool getStateGrasp() { return state_grasp_; }

		protected:
			ControlVar emg_cmd_flexion_;
			ControlVar emg_cmd_extension_;
			ControlVar force_detection_;

			// Channel locking variables
			bool emg0_lock_ = false;
			float emg0_prev_ = 0.0;
			bool emg1_lock_ = false;
			float emg1_prev_ = 0.0;

			// State auto:=true corresponds to "auto" mode, while move:=false corresponds to "manual" mode
			bool state_auto_ = true;
			bool flag_switch_ = false;
			// State rotation:=true corresponds to pro/sup movements, rotation:=false corresponds to 
			bool state_rotate_ = false;
			bool flag_coactiv_ = false;
			// State grasp:=false corresponds to grasp "force" detected
			bool state_grasp_ = false;

			ControlVar hand_supination_angle_;
			ControlVar hand_grasp_size_;

			ControlVar target_grasp_size_;
			ControlVar target_tilt_angle_;
			ControlVar target_grasp_type_;

			/* Receives a Primitive3 object and evaluates its type. */
			Primitive3Type find_primitive3_type(robin::Primitive3* prim);

			/* Estimation of the "grasp size" from a primitive3 object. */
			void estimate_grasp_size(robin::Primitive3* prim);

			/* Estimation of the "grasp type" from a primitive3 object. */
			void estimate_grasp_type(robin::Primitive3* prim);

			/* Estimation of the "tilt angle" from a primitive3 object. */
			void estimate_tilt_angle(robin::Primitive3* prim);						
		};
	}
}