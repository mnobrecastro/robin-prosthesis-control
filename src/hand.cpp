#include "hand.h"

namespace robin
{	
	namespace hand
	{
		Hand::Hand(bool right_hand) :
			right_hand_(right_hand) {}

		Hand::~Hand() {}

		bool Hand::isRightHand() { return right_hand_; }

		void Hand::setControlMode(CONTROL ctrl) { configstate_.ctrl_type = ctrl;	}

		void Hand::setGrasp(GRASP grasp) { configstate_.grasp_type = grasp; }

		float Hand::getWristFleExtAngle()
		{
			return configstate_.fle_ext_angle;
		}

		float Hand::getWristAbdAddAngle()
		{
			return configstate_.abd_add_angle;
		}

		float Hand::getWristSupProAngle()
		{
			return configstate_.sup_pro_angle;
		}

		float Hand::getGraspSize()
		{
			return configstate_.grasp_size;
		}

		float Hand::getGraspForce()
		{
			return configstate_.grasp_force;
		}

	}
	
}