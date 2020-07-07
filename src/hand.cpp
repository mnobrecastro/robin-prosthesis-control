#include "hand.h"

namespace robin
{	
	namespace hand
	{
		Hand::Hand() {}

		Hand::~Hand() {}

		void Hand::setControlMode(CONTROL ctrl)
		{
			switch (ctrl) {
			case CONTROL::POSITION:
				break;
			case CONTROL::VELOCITY:
				break;
			case CONTROL::FORCE:
				break;
			}
		}

		void Hand::setGrasp(GRASP grasp)
		{
			switch (grasp) {
			case GRASP::PALMAR:
				break;
			case GRASP::LATERAL:
				break;
			}
		}
	}
	
}