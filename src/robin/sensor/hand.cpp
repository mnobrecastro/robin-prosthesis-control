/*
 * Semi-autonomous Prosthesis Control Using Computer Vision - Robin C++ framework
 *
 * Author: Miguel Nobre Castro (mnobrecastro@gmail.com)
 *
 *
 * This work was performed at the Department of Health Science and Technology, Aalborg
 * University, under the supervision of Professor Strahinja Dosen (sdosen@hst.aau.dk),
 * and was supported by the Independent Research Fund Denmark through the project ROBIN
 * "RObust Bidirectional human-machine INterface for natural control and feedback in
 * hand prostheses" (8022-00243A).
 */

#include "hand.h"

namespace robin
{	
	namespace hand
	{
		Hand::Hand(bool right_hand) :
			right_hand_(right_hand) {}

		Hand::~Hand() {}

		bool Hand::isRightHand() { return right_hand_; }

		void Hand::setControlMode(CONTROL ctrl) { configstate_.ctrl_type = ctrl; }

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