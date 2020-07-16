#include "hand.h"

namespace robin
{	
	namespace hand
	{
		Hand::Hand(bool right_hand) :
			right_hand_(right_hand) {}

		Hand::~Hand() {}

		bool Hand::isRightHand() { return right_hand_; }

		void Hand::setControlMode(CONTROL ctrl) { configstate_.ctrl = ctrl;	}

		void Hand::setGrasp(GRASP grasp) { configstate_.grasp = grasp; }
		
	}
	
}