#include "control_simple.h"

namespace robin
{
	namespace control
	{
		ControlSimple::ControlSimple(robin::hand::Hand& hand)
		{
			hand_ = &hand;
		}

		void ControlSimple::evaluate(robin::Primitive3* prim)
		{
			//hand_->updateConfigState();

			this->estimate_grasp_size(prim);
			this->estimate_tilt_angle(prim);

			float grasp_size_error(grasp_size_.value - hand_->getGraspSize());
			if (grasp_size_error > 0) {
				hand_->open(0.0);
			}
			else {
				hand_->close(0.0);
			}

			float tilt_angle_error;
			if (hand_->isRightHand()) {
				// Right-hand prosthesis (positive tilt angle)
				tilt_angle_error = tilt_angle_.value - hand_->getSupinationAngle();
				if (grasp_size_error > 0) { hand_->supinate(0.0); }
				else { hand_->pronate(0.0); }
			}
			else {
				// Left-hand prosthesis (negative tilt angle)
				tilt_angle_error = tilt_angle_.value + hand_->getSupinationAngle();
				if (grasp_size_error > 0) { hand_->supinate(0.0); }
				else { hand_->pronate(0.0); }
			}		
		}

		float ControlSimple::getGraspSize()
		{
			return grasp_size_.value;
		}

		float ControlSimple::getTiltAngle()
		{
			return tilt_angle_.value;
		}

		/* Receives a Primitive3 object and evaluates its type. */
		Primitive3Type ControlSimple::find_primitive3_type(robin::Primitive3* prim)
		{
			if (typeid(*prim) == typeid(robin::Primitive3Sphere)) {
				return Primitive3Type::PRIMITIVE3_SPHERE;				
			}
			else if (typeid(*prim) == typeid(robin::Primitive3Cuboid)) {
				return Primitive3Type::PRIMITIVE3_CUBOID;				
			}
			else if (typeid(*prim) == typeid(robin::Primitive3Cylinder)) {
				return Primitive3Type::PRIMITIVE3_CYLINDER;				
			}
			else {
				std::cout << "Problem indentifying the primitive3!" << std::endl;
				return Primitive3Type::UNKNOWN;
			}
		}

		void ControlSimple::estimate_grasp_size(robin::Primitive3* prim)
		{
			float grasp_size;
			switch (find_primitive3_type(prim)){
			case Primitive3Type::PRIMITIVE3_SPHERE:
				grasp_size = prim->getProperty_radius();
				break;
			case Primitive3Type::PRIMITIVE3_CUBOID:
				if (prim->getProperty_width() > 0.001) {
					grasp_size = prim->getProperty_width();
				}
				if (prim->getProperty_height() > 0.001 && grasp_size > prim->getProperty_height()) {
					grasp_size = prim->getProperty_height();
				}
				if (prim->getProperty_depth() > 0.001 && grasp_size > prim->getProperty_depth()) {
					grasp_size = prim->getProperty_depth();
				}
				break;
			case Primitive3Type::PRIMITIVE3_CYLINDER:
				grasp_size = prim->getProperty_radius();
				break;
			}
			grasp_size_.buffer.push_back(grasp_size);
			std::cout << "Estimated grasp_size: " << grasp_size;

			// Calculate and use the median value
			std::vector<float> temp;
			size_t n_samples(100);
			if (grasp_size_.buffer.size() < n_samples) {
				temp = grasp_size_.buffer;
			} else {
				temp = std::vector<float>(grasp_size_.buffer.end() - n_samples, grasp_size_.buffer.end());
			}
			std::sort(temp.begin(), temp.begin());
			grasp_size_.value = temp[(int) temp.size()/2];
			std::cout << " with median: " << grasp_size_.value << std::endl;
		}

		void ControlSimple::estimate_tilt_angle(robin::Primitive3* prim)
		{			
			// Tilt angle calculated as an absolute supination angle, i.e. measured fromt the full pronated wrist position.
			float tilt_angle;
			Primitive3Type prim3_type = find_primitive3_type(prim);
			if (prim3_type == Primitive3Type::PRIMITIVE3_SPHERE)
			{
				/* To be further implemented. */
				tilt_angle = hand_->getSupinationAngle(); // Hand stays as it is.
			}
			else if(prim3_type == Primitive3Type::PRIMITIVE3_CUBOID || prim3_type == Primitive3Type::PRIMITIVE3_CYLINDER)
			{				
				// Angle of the projection of the Primitive3 axis into the xOy plane of the camera.
				float projected_angle(std::atan2(prim->getProperty_axis_y(), prim->getProperty_axis_x()));
				
				// Current absolute supination angle
				float supination_angle(hand_->getSupinationAngle());
				if (hand_->isRightHand()) {
					// Right-hand prosthesis (positive tilt angle)
					tilt_angle = supination_angle + projected_angle;
					if (tilt_angle < 0) { tilt_angle += M_PI; }
					else if (tilt_angle > M_PI) { tilt_angle -= M_PI; }
				}
				else {
					// Left-hand prosthesis (negative tilt angle)
					tilt_angle = -supination_angle + projected_angle;
					if (tilt_angle < -M_PI) { tilt_angle += M_PI; }
					else if (tilt_angle > M_PI) { tilt_angle -= M_PI; }
				}
			}
			tilt_angle_.buffer.push_back(tilt_angle);
			std::cout << "Estimated tilt_angle: " << tilt_angle;

			// Calculate and use the median value
			std::vector<float> temp;
			size_t n_samples(100);
			if (tilt_angle_.buffer.size() < n_samples) {
				temp = tilt_angle_.buffer;
			}
			else {
				temp = std::vector<float>(tilt_angle_.buffer.end() - n_samples, tilt_angle_.buffer.end());
			}
			std::sort(temp.begin(), temp.begin());
			tilt_angle_.value = temp[(int) temp.size()/2];
			std::cout << " with median: " << tilt_angle_.value << std::endl;
		}
	}
}