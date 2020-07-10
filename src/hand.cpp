#include "hand.h"

namespace robin
{	
	namespace hand
	{
		Hand::Hand() {}

		Hand::~Hand() {}

		void Hand::setControlMode(CONTROL ctrl) { configstate_.ctrl = ctrl;	}

		void Hand::setGrasp(GRASP grasp) { configstate_.grasp = grasp; }
		
	}
	
}