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
			std::vector<float> emg_channels = hand_->getEMG();
			// Current EMG "MOVE" command (channel 1)
			emg_cmd_move_.value = emg_channels[0];
			emg_cmd_move_.buffer.push_back(emg_channels[0]);
			// Current EMG "GRASP" command (channel 2)
			emg_cmd_grasp_.value = emg_channels[1];
			emg_cmd_grasp_.buffer.push_back(emg_channels[1]);

			// Obtain window for MOVE command
			std::vector<float> window_cmd_move;
			if (emg_cmd_move_.buffer.size() < 100) { //&& !emg_cmd_move_.buffer.empty()
				window_cmd_move = std::vector<float>(emg_cmd_move_.buffer.begin(), emg_cmd_move_.buffer.end());
			} else if(emg_cmd_move_.buffer.size() >= 100) { //&& !emg_cmd_move_.buffer.empty()
				window_cmd_move = std::vector<float>(emg_cmd_move_.buffer.end()-99, emg_cmd_move_.buffer.end());
			}
			size_t window_size_move = window_cmd_move.size();
			std::cout << "WINDOW MOVE SIZE: " << window_size_move << std::endl;
			// Calculate median value
			std::sort(window_cmd_move.begin(), window_cmd_move.end());
			float median_cmd_move(0.0);
			if (window_size_move % 2 == 0) {
				median_cmd_move = (window_cmd_move[window_size_move/2-1] + window_cmd_move[window_size_move/2]) / 2;
			} else {
				median_cmd_move = window_cmd_move[window_size_move/2];
			}
			// Set current MOVE command
			bool usr_cmd_move(false);
			if (emg_cmd_move_.value >= 0.75) { //median_cmd_move
				usr_cmd_move = true;
			}


			// Obtain window for GRASP command
			std::vector<float> window_cmd_grasp;
			if (emg_cmd_grasp_.buffer.size() < 100) { //&& !emg_cmd_grasp_.buffer.empty()
				window_cmd_grasp = std::vector<float>(emg_cmd_grasp_.buffer.begin(), emg_cmd_grasp_.buffer.end());
			}
			else if (emg_cmd_grasp_.buffer.size() >= 100) { //&& !emg_cmd_grasp_.buffer.empty()
				window_cmd_grasp = std::vector<float>(emg_cmd_grasp_.buffer.end() - 99, emg_cmd_grasp_.buffer.end());
			}
			size_t window_size_grasp = window_cmd_grasp.size();
			std::cout << "WINDOW GRASP SIZE: " << window_size_grasp << std::endl;
			// Calculate median value
			std::sort(window_cmd_grasp.begin(), window_cmd_grasp.end());
			float median_cmd_grasp(0.0);
			if (window_size_grasp % 2 == 0) {
				median_cmd_grasp = (window_cmd_grasp[window_size_grasp/2-1] + window_cmd_grasp[window_size_grasp/2]) / 2;
			}
			else {
				median_cmd_grasp = window_cmd_grasp[window_size_grasp/2];
			}
			// Set current GRASP command
			bool usr_cmd_grasp(false);
			if (emg_cmd_grasp_.value >= 0.75) { //median_cmd_grasp
				usr_cmd_grasp = true;
			}

			// Change the MOVE and GRASP state booleans
			//if (usr_cmd_move) {
			//	if (state_move_ && !state_grasp_) { state_move_ = false; }
			//	if (!state_move_ && !state_grasp_) { state_move_ = true; }
			//}
			////else if (usr_cmd_grasp) { // "else if" to prevent simultaneous behaviour
			////	if (!state_grasp_ && !state_move_) { state_grasp_ = true; }
			////	if (state_grasp_ && !state_move_) { state_grasp_ = false; }
			////}
			if (usr_cmd_move) {
				state_move_ = !state_move_;
			}

			// ----
			
			// Current absolute supination angle of the prosthesis
			// (measured from the full pronated wrist position ref frame).
			//
			//			                 (Z)
			//                           /
			//                          /
			//		                   /_ _ _ _ (X)
			//                         |
			//                  A      |      A
			// (Sup Right-hand) |__    |    __| (Sup Left-hand) 
			//                        (Y)
			//
			float supination_angle;
			if (hand_->isRightHand()) {
				// Right-hand prosthesis (positive angle)
				supination_angle = hand_->getWristSupProAngle();
			}
			else {
				// Left-hand prosthesis (negative angle)
				supination_angle = -hand_->getWristSupProAngle();
			}
			hand_supination_angle_.value = supination_angle;
			hand_supination_angle_.buffer.push_back(supination_angle);

			// Current grasp size of the prosthesis
			hand_grasp_size_.value = hand_->getGraspSize();
			hand_grasp_size_.buffer.push_back(hand_->getGraspSize());


			this->estimate_grasp_size(prim);
			this->estimate_grasp_type(prim);
			this->estimate_tilt_angle(prim);
						
			// Tolerances
			float grasp_size_error_tol(0.01);
			float tilt_angle_error_tol(10 * M_PI/180);

			if (state_move_)
			{
				float grasp_size_error(target_grasp_size_.value - hand_grasp_size_.value);
				if (std::abs(grasp_size_error) > grasp_size_error_tol) {
					if (grasp_size_error > 0.0) {
						hand_->open(0.01, false); //*
					}
					else {
						hand_->close(target_grasp_type_, 0.01, false); //*
					}
				}
				else {
					hand_->open(0.0, false); //stop() //*
				}

				float tilt_angle_error(target_tilt_angle_.value - hand_supination_angle_.value);
				if (hand_->isRightHand()) {
					// Right-hand prosthesis (positive tilt angle)				
					if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
						if (tilt_angle_error > 0.0) {
							hand_->supinate(0.01, false);
						}
						else {
							hand_->pronate(0.01, false);
						}
					}
					else {
						hand_->supinate(0.0, false);
					}
				}
				else {
					// Left-hand prosthesis (negative tilt angle)
					if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
						if (tilt_angle_error < 0.0) {
							hand_->supinate(0.01, false);
						}
						else {
							hand_->pronate(0.01, false);
						}
					}
					else {
						hand_->supinate(0.0, false);
					}
				}
			}
			else {
				if (!state_grasp_) {
					if (!usr_cmd_grasp) {
						hand_->stop();
					} else {
						hand_->close(target_grasp_type_, 0.01, false);
						state_grasp_ = !state_grasp_;
					}
				}
				else{
					if(usr_cmd_grasp){
						hand_->open(0.01, false);
						state_grasp_ = !state_grasp_;
					}else{
						hand_->stop();
					}
				}				
			}
			hand_->send_command();
		}

		float ControlSimple::getGraspSize()
		{
			return target_grasp_size_.value;
		}

		float ControlSimple::getTiltAngle()
		{
			return target_tilt_angle_.value;
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
			// Cuboid vars
			float projection(0.0);
			Eigen::Vector3f axis, cam_axis(0.0, 0.0, 1.0), e0_axis, e1_axis, z_axis;
			//

			float grasp_size(10.0);
			switch (find_primitive3_type(prim)){
			case Primitive3Type::PRIMITIVE3_SPHERE:
				grasp_size = 2.0 * prim->getProperty_radius();
				break;
			case Primitive3Type::PRIMITIVE3_CUBOID:				
				// Original variant (pick the smallest dimension)
				/*if (prim->getProperty_width() > 0.005) {
					grasp_size = prim->getProperty_width();
				}
				if (prim->getProperty_height() > 0.005 && grasp_size > prim->getProperty_height()) {
					grasp_size = prim->getProperty_height();
				}
				if (prim->getProperty_depth() > 0.005 && grasp_size > prim->getProperty_depth()) {
					grasp_size = prim->getProperty_depth();
				}*/

				// Projection variant (closest normal to cam-axis)
				e0_axis = { prim->getProperty_e0_x(), prim->getProperty_e0_y(), prim->getProperty_e0_z() };
				e1_axis = { prim->getProperty_e1_x(), prim->getProperty_e1_y(), prim->getProperty_e1_z() };
				z_axis = { prim->getProperty_axis_x(), prim->getProperty_axis_y(), prim->getProperty_axis_z() };

				if (projection < std::abs(e0_axis.dot(cam_axis) / (e0_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(e0_axis.dot(cam_axis) / (e0_axis.norm() * cam_axis.norm()));
					if (e1_axis.norm() >= z_axis.norm()) {
						grasp_size = 2.0 * z_axis.norm();
					}
					else {
						grasp_size = 2.0 * e1_axis.norm();
					}
				}
				if (projection < std::abs(e1_axis.dot(cam_axis) / (e1_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(e1_axis.dot(cam_axis) / (e1_axis.norm() * cam_axis.norm()));
					if (e0_axis.norm() >= z_axis.norm()) {
						grasp_size = 2.0 * z_axis.norm();
					}
					else {
						grasp_size = 2.0 * e0_axis.norm();
					}
				}
				if (projection < std::abs(z_axis.dot(cam_axis) / (z_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(z_axis.dot(cam_axis) / (z_axis.norm() * cam_axis.norm()));
					if (e0_axis.norm() >= e1_axis.norm()) {
						grasp_size = 2.0 * e1_axis.norm();
					}
					else {
						grasp_size = 2.0 * e0_axis.norm();
					}
				}
				break;
			case Primitive3Type::PRIMITIVE3_CYLINDER:
				grasp_size = 2.0 * prim->getProperty_radius();
				break;
			}
			target_grasp_size_.buffer.push_back(grasp_size);
			std::cout << "Estimated grasp_size: " << grasp_size;


			// Calculate and use the median value
			std::vector<float> temp;
			size_t n_samples(10);
			if (target_grasp_size_.buffer.size() < n_samples) {
				temp = target_grasp_size_.buffer;
			} else {
				temp = std::vector<float>(target_grasp_size_.buffer.end() - n_samples, target_grasp_size_.buffer.end());
			}
			std::sort(temp.begin(), temp.begin());
			target_grasp_size_.value = temp[(int) temp.size()/2];
			std::cout << " with median: " << target_grasp_size_.value << std::endl;
		}

		void ControlSimple::estimate_grasp_type(robin::Primitive3* prim)
		{
			float length_threshold(0.080);
			float grasp_size_threshold(0.050);
			float projection_threshold(std::cos(M_PI/6)); //30deg

			// Cuboid vars
			float projection(0.0);
			Eigen::Vector3f axis, cam_axis(0.0, 0.0, 1.0), e0_axis, e1_axis, z_axis;
			//

			robin::hand::GRASP grasp_type(robin::hand::GRASP::PALMAR);
			switch (find_primitive3_type(prim)) {
			case Primitive3Type::PRIMITIVE3_SPHERE:
				if (target_grasp_size_.value < grasp_size_threshold) {
					grasp_type = robin::hand::GRASP::LATERAL;
				}
				else {
					grasp_type = robin::hand::GRASP::PALMAR;
				}
				break;

			case Primitive3Type::PRIMITIVE3_CUBOID:
				// Projection variant (closest normal to cam-axis)				
				e0_axis = { prim->getProperty_e0_x(), prim->getProperty_e0_y(), prim->getProperty_e0_z() };
				e1_axis = { prim->getProperty_e1_x(), prim->getProperty_e1_y(), prim->getProperty_e1_z() };
				z_axis = { prim->getProperty_axis_x(), prim->getProperty_axis_y(), prim->getProperty_axis_z() };

				if (projection < std::abs(e0_axis.dot(cam_axis) / (e0_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(e0_axis.dot(cam_axis) / (e0_axis.norm() * cam_axis.norm()));
					if (e1_axis.norm() >= z_axis.norm()) {
						axis = e1_axis;
					} else {
						axis = z_axis;
					}					
				}
				if (projection < std::abs(e1_axis.dot(cam_axis) / (e1_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(e1_axis.dot(cam_axis) / (e1_axis.norm() * cam_axis.norm()));
					if (e0_axis.norm() >= z_axis.norm()) {
						axis = e0_axis;
					}
					else {
						axis = z_axis;
					}
				}
				if (projection < std::abs(z_axis.dot(cam_axis) / (z_axis.norm() * cam_axis.norm())))
				{
					projection = std::abs(z_axis.dot(cam_axis) / (z_axis.norm() * cam_axis.norm()));
					if (e0_axis.norm() >= e1_axis.norm()) {
						axis = e0_axis;
					}
					else {
						axis = e1_axis;
					}
				}

				if (2.0 * axis.norm() < length_threshold) {
					if (target_grasp_size_.value < grasp_size_threshold) {
						grasp_type = robin::hand::GRASP::LATERAL;
					} else {
						grasp_type = robin::hand::GRASP::PALMAR;
					}
				}
				else {
					grasp_type = robin::hand::GRASP::PALMAR;
				}
				break;

			case Primitive3Type::PRIMITIVE3_CYLINDER:
				axis = { prim->getProperty_axis_x(), prim->getProperty_axis_y(), prim->getProperty_axis_z() };
				if (2.0 * axis.norm() < length_threshold) {
					if (target_grasp_size_.value < grasp_size_threshold) {
						grasp_type = robin::hand::GRASP::LATERAL;
					}else {
						//Eigen::Vector3f cam_axis(0.0, 0.0, 1.0);
						float projection = std::abs( axis.dot(cam_axis)/(axis.norm()*cam_axis.norm()) );
						if (projection < projection_threshold) {
							grasp_type = robin::hand::GRASP::LATERAL;
						} else {
							grasp_type = robin::hand::GRASP::PALMAR;
						}
					}
				} else {
					grasp_type = robin::hand::GRASP::PALMAR;
				}
				break;
			}
			target_grasp_type_ = grasp_type;
			std::cout << "Estimated grasp_type: ";
			switch (grasp_type) {
			case robin::hand::GRASP::PALMAR:
				std::cout << "PALMAR" << std::endl;
				break;
			case robin::hand::GRASP::LATERAL:
				std::cout << "LATERAL" << std::endl;
				break;
			}
		}

		void ControlSimple::estimate_tilt_angle(robin::Primitive3* prim)
		{						
			// Tilt angle calculated as an absolute supination angle, i.e. measured from the full pronated wrist position.
			float tilt_angle;
			Primitive3Type prim3_type = find_primitive3_type(prim);
			if (prim3_type == Primitive3Type::PRIMITIVE3_SPHERE)
			{
				/* To be further implemented. */
				tilt_angle = hand_supination_angle_.value; // Hand stays as it is.
			}
			else if (prim3_type == Primitive3Type::PRIMITIVE3_CUBOID || prim3_type == Primitive3Type::PRIMITIVE3_CYLINDER)
			{
				// Angle of the projection of the Primitive3 axis into the xOy plane of the camera.
				float projected_angle(std::atan2(prim->getProperty_axis_y(), prim->getProperty_axis_x()));

				if (target_grasp_type_ == robin::hand::GRASP::PALMAR) {
					// Correction of the projection angle w.r.t. the hand-side (axis may have been fliped or not)
					if (hand_->isRightHand()) {
						// Right-hand prosthesis (positive angle)
						if (hand_supination_angle_.value + projected_angle < 0) { projected_angle += M_PI; }
						else if (hand_supination_angle_.value + projected_angle > M_PI) { projected_angle -= M_PI; }

					}
					else {
						// Left-hand prosthesis (negative angle)
						if (hand_supination_angle_.value + projected_angle > 0) { projected_angle -= M_PI; }
						else if (hand_supination_angle_.value + projected_angle < -M_PI) { projected_angle += M_PI; }
					}

					// Current absolute tilt angle of the Primitive3
					tilt_angle = hand_supination_angle_.value + projected_angle;
				}
				else {
					// Correction of the projection angle w.r.t. the hand-side (axis may have been fliped or not)
					if (hand_->isRightHand()) {
						// Right-hand prosthesis (positive angle)
						if (hand_supination_angle_.value + projected_angle + M_PI/2 < 0) { projected_angle += M_PI; }
						else if (hand_supination_angle_.value + projected_angle + M_PI/2 > M_PI) { projected_angle -= M_PI; }

					}
					else {
						// Left-hand prosthesis (negative angle)
						if (hand_supination_angle_.value + projected_angle + M_PI/2 > 0) { projected_angle -= M_PI; }
						else if (hand_supination_angle_.value + projected_angle + M_PI/2 < -M_PI) { projected_angle += M_PI; }
					}

					// Current absolute tilt angle of the Primitive3
					tilt_angle = hand_supination_angle_.value + projected_angle + M_PI/2;
				}
			}
			target_tilt_angle_.buffer.push_back(tilt_angle);
			std::cout << "Estimated tilt_angle: " << tilt_angle;


			// Calculate and use the median value
			std::vector<float> temp;
			size_t n_samples(10);
			if (target_tilt_angle_.buffer.size() < n_samples) {
				temp = target_tilt_angle_.buffer;
			}
			else {
				temp = std::vector<float>(target_tilt_angle_.buffer.end() - n_samples, target_tilt_angle_.buffer.end());
			}
			std::sort(temp.begin(), temp.begin());
			target_tilt_angle_.value = temp[(int) temp.size()/2];
			std::cout << " with median: " << target_tilt_angle_.value << std::endl;
		}
	}
}