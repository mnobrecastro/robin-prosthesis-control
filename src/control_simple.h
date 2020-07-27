#pragma once
#include "control.h"
#include "primitive3.h"
#include "primitive3_sphere.h"
#include "primitive3_cuboid.h"
#include "primitive3_cylinder.h"

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

		protected:
			ControlVar hand_supination_angle_;
			ControlVar hand_grasp_size_;

			ControlVar target_grasp_size_;
			ControlVar target_tilt_angle_;

			/* Receives a Primitive3 object and evaluates its type. */
			Primitive3Type find_primitive3_type(robin::Primitive3* prim);

			/* Estimation of the "grasp size" from a primitive3 object. */
			void estimate_grasp_size(robin::Primitive3* prim);

			/* Estimation of the "tilt angle" from a primitive3 object. */
			void estimate_tilt_angle(robin::Primitive3* prim);						
		};
	}
}