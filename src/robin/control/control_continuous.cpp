#include "control_continuous.h"

namespace robin
{
	namespace control
	{
		ControlContinuous::ControlContinuous(robin::hand::Hand& hand)
		{
			hand_ = &hand;

			emg0_time_ = std::chrono::high_resolution_clock::now();
			emg1_time_ = emg0_time_;
		}

		void ControlContinuous::evaluate(robin::Primitive3* prim)
		{
			//std::vector<float> emg_channels = hand_->getEMG();
			std::vector<Solver1EMG*> emg_channels = dynamic_cast<robin::hand::Michelangelo*>(hand_)->getEMGSolvers();
			// Current EMG "flexion" command (channel 1)
			emg_cmd_flexion_.buffer.push_back(emg_channels[0]->getMean(15));
			emg_cmd_flexion_.update(robin::control::ControlVar::fname::MOVING_AVERAGE, 1);
			// Current EMG "extension" command (channel 2)
			emg_cmd_extension_.buffer.push_back(emg_channels[1]->getMean(15));
			emg_cmd_extension_.update(robin::control::ControlVar::fname::MOVING_AVERAGE, 1);

			// Current Force measure
			force_detection_.buffer.push_back(hand_->getGraspForce());
			force_detection_.update(robin::control::ControlVar::fname::MOVING_AVERAGE, 10);

			// Interpret user commands
			float emg_minimal_threshold(0.1); // [0.0-1.0]
			float emg_command_threshold(0.7); // [0.0-1.0]
			float emg_coactiv_threshold(0.5); // [0.0-1.0]

			float hand_velocity(0.5); // [0.0-1.0]
			float force_threshold(5.0); // [N]		
			

			////////////////////////////////////////////////////////////////////////////////////////////////

			int window_size = 15; // 150ms @ 100hz
			float act_ch0 = emg_channels[0]->getMean(window_size);
			float act_ch1 = emg_channels[1]->getMean(window_size);

			//// Check for COACTIVATION
			bool usr_cmd_myomenu(false);
			if (!state_auto_) //if (!flag_emg0_ && !flag_emg1_)
			{
				if (!flag_coactiv_ && (act_ch0 > emg_coactiv_threshold && act_ch1 > emg_coactiv_threshold)) {
					// Flag raised
					flag_coactiv_ = true;
				}
				else if (flag_coactiv_ && (act_ch0 < emg_minimal_threshold && act_ch1 < emg_minimal_threshold)) {
					// Flag lowered
					flag_coactiv_ = false;
					// Cmd Myomenu
					usr_cmd_myomenu = true;
				}
			}
						
			//// Check for EMG COMMANDs
			bool usr_cmd_ch0(false);
			bool usr_cmd_ch1(false);
			if (state_auto_) //if (!flag_coactiv_)
			{				
				if (!flag_emg1_) {
					// EMG Ch0
					if (!flag_emg0_ && act_ch0 > emg_command_threshold) {
						flag_emg0_ = true;
					}
					else if (flag_emg0_ && act_ch0 < emg_minimal_threshold) {
						flag_emg0_ = false;
						usr_cmd_ch0 = true;
					}
				}
				if (!flag_emg0_) {
					// EMG Ch1
					if (!flag_emg1_ && act_ch1 > emg_command_threshold) {
						flag_emg1_ = true;
					}
					else if (flag_emg1_ && act_ch1 < emg_minimal_threshold) {
						flag_emg1_ = false;
						usr_cmd_ch1 = true;
					}
				}
			}

			std::cout << ">>>>>>  EMG0: " << flag_emg0_
				<< " EMG1: " << flag_emg1_
				<< " COACT: " << flag_coactiv_ << '\n';

			// Set current GRASP command
			bool hand_cmd_grasp(false);
			if (force_detection_.value >= force_threshold) { // [N]
				hand_cmd_grasp = true;
			}

			std::cout << "******  EMG0: " << emg_cmd_flexion_.value 
				<< " EMG1: " << emg_cmd_extension_.value 
				<< " FORCE: " << force_detection_.value << '\n';

			// ----

			// Check whether full manual control has been set
			if (full_manual_) {
				state_auto_ = false;
			}

			/* The prosthetic hand automatically starts in "auto" mode, being the user able to
			 * switch to "manual" mode. Consequently, the "manual" mode starts in "Open/Close"
			 * operation, which can also be alternated to "Supination/Pronation" upon user input.
			 */

			//// Checks if a key has been pressed to RESET the hand position
			char key = '#';
			bool pressed = isKeyPressed(&key);
			if (pressed && (key == ' ' || key == 'v' || key == 's')) {
				key_stored_ = key;
				key_pressed_ = true;
				flag_key_ = true;

				state_auto_ = true;
			}
			else if (key_pressed_ && !flag_key_) {
				key_pressed_ = false;

				if (full_manual_) {
					state_auto_ = false;
				}
			}			

			// ***************************************************************************
			if (state_auto_)
			{
				if (usr_cmd_ch1) {
					// Save event to DataManager (double)
					this->saveEvent();

					hand_->stop();
					state_auto_ = false;

					// Change the target_grasp_type_ accordingly
					switch (static_cast<robin::hand::GRASP>(int(target_grasp_type_.value))) {
					case robin::hand::GRASP::PALMAR:
						state_myomenu_ = MyocontrolMenu::MYOMENU_PALMAR;
						break;
					case robin::hand::GRASP::LATERAL:
						state_myomenu_ = MyocontrolMenu::MYOMENU_LATERAL;
						break;
					default:
						/* do nothing */
						break;
					}

					// Save event to DataManager
					this->saveEvent();

					//usr_cmd_stop = false;
					Beep(2000, 100);
				}
				else {
					/* Current absolute supination angle of the prosthesis
					 * (measured from the full pronated wrist position ref frame).
					 *
					 *			                 (Z)
					 *                           /
					 *                          /
					 *		                   /_ _ _ _ (X)
					 *                         |
					 *                  A      |      A
					 * (Sup Right-hand) |__    |    __| (Sup Left-hand)
					 *                        (Y)
					 */
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
					float hand_grasp_size = hand_->getGraspSize();
					hand_grasp_size_.value = hand_grasp_size;
					hand_grasp_size_.buffer.push_back(hand_grasp_size);


					this->estimate_grasp_size(prim);
					this->estimate_grasp_type(prim);
					this->estimate_tilt_angle(prim);

					if (flag_key_) {
						switch (key_stored_) {
						case ' ':
							// Set target_grasp_type to GRASP::PALMAR
							target_grasp_type_.value = int(robin::hand::GRASP::PALMAR);
							// Set taget_tilt_angle to neutral
							target_tilt_angle_.value = 0.0;
							// Set target_grasp_size to full opened hand
							target_grasp_size_.value = 0.10;
							break;

						case 'v':
							// Set target_grasp_type to GRASP::PALMAR
							target_grasp_type_.value = int(robin::hand::GRASP::PALMAR);
							// Set taget_tilt_angle to 90 deg
							if (hand_->isRightHand())
								target_tilt_angle_.value = M_PI / 2.0;
							else
								target_tilt_angle_.value = -M_PI / 2.0;
							// Set target_grasp_size to full opened hand
							target_grasp_size_.value = 0.10;
							break;

							//case 's':
							//	if (sequential) {
							//		this->estimate_grasp_size(prim);
							//		this->estimate_grasp_type(prim);
							//		this->estimate_tilt_angle(prim);
							//	}
							//	break;
						}
					}

					// Tolerances
					float grasp_size_slack(0.030); // 3 cm
					float grasp_size_error_tol(0.010); // 1 cm
					float tilt_angle_error_tol(5 * M_PI / 180);


					float grasp_size_error(target_grasp_size_.value + grasp_size_slack - hand_grasp_size_.value);
					if (std::abs(grasp_size_error) > grasp_size_error_tol) {
						if (grasp_size_error > 0.0) {
							hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), hand_velocity, false); //* hand_velocity/2
						}
						else {
							hand_->close(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), hand_velocity, false); //* hand_velocity/2
						}
					}
					else {
						hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), 0.0, false);
					}

					float tilt_angle_error(target_tilt_angle_.value - hand_supination_angle_.value);
					if (hand_->isRightHand()) {
						// Right-hand prosthesis (positive tilt angle)				
						if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
							if (tilt_angle_error > 0.0) {
								hand_->supinate(hand_velocity, false);
							}
							else {
								hand_->pronate(hand_velocity, false);
							}
						}
						else {
							hand_->supinate(0.0, false);

							// Upon key_pressed
							if (key_pressed_ && flag_key_) {
								flag_key_ = false;
							}
						}
					}
					else {
						// Left-hand prosthesis (negative tilt angle)
						if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
							if (tilt_angle_error < 0.0) {
								hand_->supinate(hand_velocity, false);
							}
							else {
								hand_->pronate(hand_velocity, false);
							}
						}
						else {
							hand_->supinate(0.0, false);

							// Upon key_pressed
							if (key_pressed_ && flag_key_) {
								flag_key_ = false;
							}
						}
					}
				}
			}
			else {
					
				// Checks if a rotate cmd has been triggered
				if (usr_cmd_myomenu) {
					hand_->stop();
					if (state_myomenu_ == MyocontrolMenu::MYOMENU_ROTATE) {
						state_myomenu_ = MyocontrolMenu::MYOMENU_PALMAR;
					}
					else {
						state_myomenu_ = static_cast<MyocontrolMenu>(int(state_myomenu_) + 1);
					}
					usr_cmd_myomenu = false;


					// Change the target_grasp_type_ accordingly
					switch (state_myomenu_) {
					case MyocontrolMenu::MYOMENU_PALMAR:
						target_grasp_type_.buffer.push_back(int(robin::hand::GRASP::PALMAR));
						target_grasp_type_.value = target_grasp_type_.buffer.back(); // Direct assignement without var.update()
						break;
					case MyocontrolMenu::MYOMENU_LATERAL:
						target_grasp_type_.buffer.push_back(int(robin::hand::GRASP::LATERAL));
						target_grasp_type_.value = target_grasp_type_.buffer.back(); // Direct assignement without var.update()
						break;
					default:
						/* do nothing */
						break;
					}

					// Save event to DataManager
					this->saveEvent();

					Beep(2000, 100); Beep(2000, 100);
				}

				if (state_myomenu_ != MyocontrolMenu::MYOMENU_ROTATE) {
					// The hand is in Manual [Open/Close] mode
					if (!flag_coactiv_)
					{
						if (emg_cmd_flexion_.value > emg_minimal_threshold || emg_cmd_extension_.value > emg_minimal_threshold)
						{
							if (emg_cmd_flexion_.value >= emg_cmd_extension_.value) {
								hand_->close(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), (emg_cmd_flexion_.value - emg_minimal_threshold) / (1.0 - emg_minimal_threshold), false);
							}
							else {
								// Checks if the force sensor has been activated
								if (hand_cmd_grasp) {
									// Successful grasp completion - reset all state variables

									// Save event to DataManager (double)
									this->saveEvent();

									hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), 0.5, true); // ("true" forces the command to be excuted right away)
									state_auto_ = true;

									/*// Change the grasp_type in case of full manual mode
									if (full_manual_) {
										if (static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)) == robin::hand::GRASP::PALMAR) {
											target_grasp_type_.buffer.push_back(int(robin::hand::GRASP::LATERAL));
											target_grasp_type_.value = target_grasp_type_.buffer.back(); // Direct assignement without var.update()
										}
										else {
											target_grasp_type_.buffer.push_back(int(robin::hand::GRASP::PALMAR));
											target_grasp_type_.value = target_grasp_type_.buffer.back(); // Direct assignement without var.update()
										}
									}*/

									// Save event to DataManager
									this->saveEvent(true);

									Beep(523, 100);
									std::this_thread::sleep_for(std::chrono::milliseconds(5000));
								}
								else {
									hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), (emg_cmd_extension_.value - emg_minimal_threshold) / (1.0 - emg_minimal_threshold), false);

									// Save event to DataManager
									this->saveEvent();
								}
							}
						}
						else {
							// Stop moving the hand in case no EMG signal is recorded
							hand_->stop();
						}
					}
				}
				else {
					// The hand is in Manual [Supination/Pronation] mode
					if (!flag_coactiv_)
					{
						if (emg_cmd_flexion_.value > emg_minimal_threshold || emg_cmd_extension_.value > emg_minimal_threshold)
						{
							if (emg_cmd_flexion_.value >= emg_cmd_extension_.value) {
								hand_->pronate((emg_cmd_flexion_.value - emg_minimal_threshold) / (1.0 - emg_minimal_threshold), false);

								// Save event to DataManager
								this->saveEvent();
							}
							else {
								hand_->supinate((emg_cmd_extension_.value - emg_minimal_threshold) / (1.0 - emg_minimal_threshold), false);

								// Save event to DataManager
								this->saveEvent();
							}
						}
						else {
							// Stop moving the hand in case no EMG signal is recorded
							hand_->stop();
						}
					}
				}
				//}
			}
			hand_->send_command();



			//////////////////////////////////////////////////////////////////////////////////////////////////
			//if (state_auto_) {
			//	// Semi-autonomous control

			//	// Tolerances
			//	float grasp_size_slack(0.030); // 3 cm
			//	float grasp_size_error_tol(0.010); // 1 cm
			//	float tilt_angle_error_tol(5 * M_PI / 180);

			//	// GRASP SIZE AND TYPE CONTROLLER
			//	float grasp_size_error(target_grasp_size_.value + grasp_size_slack - hand_grasp_size_.value);
			//	if (std::abs(grasp_size_error) > grasp_size_error_tol) {
			//		if (grasp_size_error > 0.0) {
			//			hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), hand_velocity, false); //* hand_velocity/2
			//		}
			//		else {
			//			hand_->close(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), hand_velocity, false); //* hand_velocity/2
			//		}
			//	}
			//	else {
			//		hand_->open(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)), 0.0, false);
			//	}

			//	// WRIST ORIENTATION CONTROLLER
			//	float tilt_angle_error(target_tilt_angle_.value - hand_supination_angle_.value);
			//	if (hand_->isRightHand()) {
			//		// Right-hand prosthesis (positive tilt angle)				
			//		if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
			//			if (tilt_angle_error > 0.0) {
			//				hand_->supinate(hand_velocity, false);
			//			}
			//			else {
			//				hand_->pronate(hand_velocity, false);
			//			}
			//		}
			//		else {
			//			hand_->supinate(0.0, false);

			//			// Upon key_pressed
			//			if (key_pressed_ && flag_key_) {
			//				flag_key_ = false;
			//			}
			//		}
			//	}
			//	else {
			//		// Left-hand prosthesis (negative tilt angle)
			//		if (std::abs(tilt_angle_error) > tilt_angle_error_tol) {
			//			if (tilt_angle_error < 0.0) {
			//				hand_->supinate(hand_velocity, false);
			//			}
			//			else {
			//				hand_->pronate(hand_velocity, false);
			//			}
			//		}
			//		else {
			//			hand_->supinate(0.0, false);

			//			// Upon key_pressed
			//			if (key_pressed_ && flag_key_) {
			//				flag_key_ = false;
			//			}
			//		}
			//	}
			//}
			//else {
			//	// Manual/Direct control

			//}
			//hand_->send_command();

		}


		float ControlContinuous::getGraspSize()
		{
			return target_grasp_size_.value;
		}

		float ControlContinuous::getTiltAngle()
		{
			return target_tilt_angle_.value;
		}

		std::vector<float> ControlContinuous::getEMG()
		{
			std::vector<float> emg;
			emg.push_back(emg_cmd_flexion_.value);
			emg.push_back(emg_cmd_extension_.value);
			return emg;
		}



		/* Receives a Primitive3 object and evaluates its type. */
		Primitive3Type ControlContinuous::find_primitive3_type(robin::Primitive3* prim)
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

		void ControlContinuous::estimate_grasp_size(robin::Primitive3* prim)
		{
			// Cuboid vars
			float projection(0.0), proj_e0_e1(0.0), proj_e0_ez(0.0), proj_e1_e0(0.0), proj_e1_ez(0.0), proj_ez_e0(0.0), proj_ez_e1(0.0);
			float d_origin_e0(0.0), d_origin_e1(0.0), d_origin_ez(0.0), d_face_e0(0.0), d_face_e1(0.0), d_face_ez(0.0);
			Eigen::Vector3f axis, cam_axis(0.0, 0.0, 1.0), e0_axis, e1_axis, z_axis, p_center;
			Eigen::Vector3f p_face_e0, p_face_e1, p_face_ez, p_inters_e0(0.0, 0.0, 0.0), p_inters_e1(0.0, 0.0, 0.0), p_inters_ez(0.0, 0.0, 0.0), vecon_e0(0.0, 0.0, 0.0), vecon_e1(0.0, 0.0, 0.0), vecon_ez(0.0, 0.0, 0.0);
			bool found(false);
			//

			float grasp_size(0.10); // target_grasp_size_.value

			if (!prim->isEmpty())
			{
				switch (find_primitive3_type(prim)) {
				case Primitive3Type::PRIMITIVE3_SPHERE:
					grasp_size = 2.0 * prim->getProperty_radius();
					break;

				case Primitive3Type::PRIMITIVE3_CUBOID:
					//// Ray casting variant
					p_center = { prim->getProperty_center_x(), prim->getProperty_center_y(), prim->getProperty_center_z() };
					e0_axis = { prim->getProperty_e0_x(), prim->getProperty_e0_y(), prim->getProperty_e0_z() };
					e1_axis = { prim->getProperty_e1_x(), prim->getProperty_e1_y(), prim->getProperty_e1_z() };
					z_axis = { prim->getProperty_axis_x(), prim->getProperty_axis_y(), prim->getProperty_axis_z() };

					// Find the closest face among each two pairs of opposing faces and calculate its center
					if (e0_axis.z() < 0) {
						p_face_e0 = p_center + e0_axis;
					}
					else {
						p_face_e0 = p_center - e0_axis;
						e0_axis *= -1;
					}
					if (e1_axis.z() < 0) {
						p_face_e1 = p_center + e1_axis;
					}
					else {
						p_face_e1 = p_center - e1_axis;
						e1_axis *= -1;
					}
					if (z_axis.z() < 0) {
						p_face_ez = p_center + z_axis;
					}
					else {
						p_face_ez = p_center - z_axis;
						z_axis *= -1;
					}

					// Find the intersection point of the cam_axis about each plane
					if (cam_axis.dot(e0_axis / e0_axis.norm()) != 0.0) {
						d_origin_e0 = p_face_e0.dot(e0_axis / e0_axis.norm()) / cam_axis.dot(e0_axis / e0_axis.norm());
						p_inters_e0 = cam_axis * d_origin_e0;
					}
					if (cam_axis.dot(e1_axis / e1_axis.norm()) != 0.0) {
						d_origin_e1 = p_face_e1.dot(e1_axis / e1_axis.norm()) / cam_axis.dot(e1_axis / e1_axis.norm());
						p_inters_e1 = cam_axis * d_origin_e1;
					}
					if (cam_axis.dot(z_axis / z_axis.norm()) != 0.0) {
						d_origin_ez = p_face_ez.dot(z_axis / z_axis.norm()) / cam_axis.dot(z_axis / z_axis.norm());
						p_inters_ez = cam_axis * d_origin_ez;
					}

					// Calculate the distance to each face center
					vecon_e0 = p_inters_e0 - p_face_e0;
					vecon_e1 = p_inters_e1 - p_face_e1;
					vecon_ez = p_inters_ez - p_face_ez;

					// Assess the following three conditions
					// 1. Calculate the projection vector to the 'inters' (intersection) point on each face
					// // face e0
					proj_e0_e1 = std::abs(vecon_e0.dot(e1_axis) / e1_axis.norm());
					proj_e0_ez = std::abs(vecon_e0.dot(z_axis) / z_axis.norm());
					std::cout << "proj_e0: " << proj_e0_e1 << "(" << e1_axis.norm() << ")" << " " << proj_e0_ez << "(" << z_axis.norm() << ")" << std::endl;
					if (!found && proj_e0_e1 <= e1_axis.norm() && proj_e0_ez <= z_axis.norm()) {
						if (e1_axis.norm() >= z_axis.norm()) {
							grasp_size = 2.0 * z_axis.norm();
						}
						else {
							grasp_size = 2.0 * e1_axis.norm();
						}
						found = true;
						prim->setIntersectionPoint(p_inters_e0);
					}
					else {
						if (proj_e0_e1 - e1_axis.norm() > 0 && proj_e0_ez - z_axis.norm() <= 0) {
							d_face_e0 = proj_e0_e1 - e1_axis.norm();
						}
						else if (proj_e0_ez - z_axis.norm() > 0 && proj_e0_e1 - e1_axis.norm() <= 0) {
							d_face_e0 = proj_e0_ez - z_axis.norm();
						}
						else if (proj_e0_e1 - e1_axis.norm() > 0 && proj_e0_ez - z_axis.norm() > 0) {
							if (proj_e0_e1 - e1_axis.norm() <= proj_e0_ez - z_axis.norm()) {
								d_face_e0 = proj_e0_e1 - e1_axis.norm();
							}
							else {
								d_face_e0 = proj_e0_ez - z_axis.norm();
							}
						}
					}
					// // face e1
					proj_e1_e0 = std::abs(vecon_e1.dot(e0_axis) / e0_axis.norm());
					proj_e1_ez = std::abs(vecon_e1.dot(z_axis) / z_axis.norm());
					std::cout << "proj_e1: " << proj_e1_e0 << "(" << e0_axis.norm() << ")" << " " << proj_e1_ez << "(" << z_axis.norm() << ")" << std::endl;
					if (!found && proj_e1_e0 <= e0_axis.norm() && proj_e1_ez <= z_axis.norm()) {
						if (e0_axis.norm() >= z_axis.norm()) {
							grasp_size = 2.0 * z_axis.norm();
						}
						else {
							grasp_size = 2.0 * e0_axis.norm();
						}
						found = true;
						prim->setIntersectionPoint(p_inters_e1);
					}
					else {
						if (proj_e1_e0 - e0_axis.norm() > 0 && proj_e1_ez - z_axis.norm() <= 0) {
							d_face_e1 = proj_e1_e0 - e0_axis.norm();
						}
						else if (proj_e1_ez - z_axis.norm() > 0 && proj_e1_e0 - e0_axis.norm() <= 0) {
							d_face_e1 = proj_e1_ez - z_axis.norm();
						}
						else if (proj_e1_e0 - e0_axis.norm() > 0 && proj_e1_ez - z_axis.norm() > 0) {
							if (proj_e1_e0 - e0_axis.norm() <= proj_e1_ez - z_axis.norm()) {
								d_face_e1 = proj_e1_e0 - e0_axis.norm();
							}
							else {
								d_face_e1 = proj_e1_ez - z_axis.norm();
							}
						}
					}
					// // face ez
					proj_ez_e0 = std::abs(vecon_ez.dot(e0_axis) / e0_axis.norm());
					proj_ez_e1 = std::abs(vecon_ez.dot(e1_axis) / e1_axis.norm());
					std::cout << "proj_ez: " << proj_ez_e0 << "(" << e0_axis.norm() << ")" << " " << proj_ez_e1 << "(" << e1_axis.norm() << ")" << std::endl;
					if (!found && proj_ez_e0 <= e0_axis.norm() && proj_ez_e1 <= e1_axis.norm()) {
						if (e0_axis.norm() >= e1_axis.norm()) {
							grasp_size = 2.0 * e1_axis.norm();
						}
						else {
							grasp_size = 2.0 * e0_axis.norm();
						}
						found = true;
						prim->setIntersectionPoint(p_inters_ez);
					}
					else {
						if (proj_ez_e0 - e0_axis.norm() > 0 && proj_ez_e1 - e1_axis.norm() <= 0) {
							d_face_ez = proj_ez_e0 - e0_axis.norm();
						}
						else if (proj_ez_e1 - e1_axis.norm() > 0 && proj_ez_e0 - e0_axis.norm() <= 0) {
							d_face_ez = proj_ez_e1 - e1_axis.norm();
						}
						else if (proj_ez_e0 - e0_axis.norm() > 0 && proj_ez_e1 - e1_axis.norm() > 0) {
							if (proj_ez_e0 - e0_axis.norm() <= proj_ez_e1 - e1_axis.norm()) {
								d_face_ez = proj_ez_e0 - e0_axis.norm();
							}
							else {
								d_face_ez = proj_ez_e1 - e1_axis.norm();
							}
						}
					}

					// 2. Select the face based on the shortest distance 'd_face'
					if (!found) {
						// // face e0
						if (p_inters_e0.z() > 0.1 && d_face_e0 <= d_face_e1 && d_face_e0 <= d_face_ez) {
							if (e1_axis.norm() >= z_axis.norm()) {
								grasp_size = 2.0 * z_axis.norm();
							}
							else {
								grasp_size = 2.0 * e1_axis.norm();
							}
							prim->setIntersectionPoint(p_inters_e0);
						}
						// // face e1
						if (p_inters_e1.z() > 0.1 && d_face_e1 <= d_face_e0 && d_face_e1 <= d_face_ez) {
							if (e0_axis.norm() >= z_axis.norm()) {
								grasp_size = 2.0 * z_axis.norm();
							}
							else {
								grasp_size = 2.0 * e0_axis.norm();
							}
							prim->setIntersectionPoint(p_inters_e1);
						}
						// // face ez
						if (p_inters_ez.z() > 0.1 && d_face_ez <= d_face_e0 && d_face_ez <= d_face_e1) {
							if (e0_axis.norm() >= e1_axis.norm()) {
								grasp_size = 2.0 * e1_axis.norm();
							}
							else {
								grasp_size = 2.0 * e0_axis.norm();
							}
							prim->setIntersectionPoint(p_inters_ez);
						}
					}
					break;

				case Primitive3Type::PRIMITIVE3_CYLINDER:
					grasp_size = 2.0 * prim->getProperty_radius();
					break;
				}
			}

			target_grasp_size_.buffer.push_back(grasp_size);
			std::cout << "Estimated grasp_size: " << grasp_size;

			target_grasp_size_.update(filter_, window_size_);
			std::cout << " with movavg: " << target_grasp_size_.value << std::endl;
		}

		void ControlContinuous::estimate_grasp_type(robin::Primitive3* prim)
		{
			float length_threshold(0.080);
			float grasp_size_threshold(0.050);
			float projection_threshold(std::cos(M_PI/6)); //30deg

			// Cuboid vars
			float projection(0.0), proj_e0_e1(0.0), proj_e0_ez(0.0), proj_e1_e0(0.0), proj_e1_ez(0.0), proj_ez_e0(0.0), proj_ez_e1(0.0);
			float d_origin_e0(0.0), d_origin_e1(0.0), d_origin_ez(0.0), d_face_e0(0.0), d_face_e1(0.0), d_face_ez(0.0);
			Eigen::Vector3f axis, cam_axis(0.0, 0.0, 1.0), e0_axis, e1_axis, z_axis, p_center;
			Eigen::Vector3f p_face_e0, p_face_e1, p_face_ez, p_inters_e0(0.0, 0.0, 0.0), p_inters_e1(0.0, 0.0, 0.0), p_inters_ez(0.0, 0.0, 0.0), vecon_e0(0.0, 0.0, 0.0), vecon_e1(0.0, 0.0, 0.0), vecon_ez(0.0, 0.0, 0.0);
			bool found(false);
			//

			robin::hand::GRASP grasp_type(static_cast<robin::hand::GRASP>(int(target_grasp_type_.value)));

			if (!prim->isEmpty())
			{
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
					//// Ray casting variant
					p_center = { prim->getProperty_center_x(), prim->getProperty_center_y(), prim->getProperty_center_z() };
					e0_axis = { prim->getProperty_e0_x(), prim->getProperty_e0_y(), prim->getProperty_e0_z() };
					e1_axis = { prim->getProperty_e1_x(), prim->getProperty_e1_y(), prim->getProperty_e1_z() };
					z_axis = { prim->getProperty_axis_x(), prim->getProperty_axis_y(), prim->getProperty_axis_z() };

					// Find the closest face among each two pairs of opposing faces and calculate its center
					if (e0_axis.z() < 0) {
						p_face_e0 = p_center + e0_axis;
					}
					else {
						p_face_e0 = p_center - e0_axis;
						e0_axis *= -1;
					}
					if (e1_axis.z() < 0) {
						p_face_e1 = p_center + e1_axis;
					}
					else {
						p_face_e1 = p_center - e1_axis;
						e1_axis *= -1;
					}
					if (z_axis.z() < 0) {
						p_face_ez = p_center + z_axis;
					}
					else {
						p_face_ez = p_center - z_axis;
						z_axis *= -1;
					}

					// Find the intersection point of the cam_axis about each plane
					if (cam_axis.dot(e0_axis / e0_axis.norm()) != 0.0) {
						d_origin_e0 = p_face_e0.dot(e0_axis / e0_axis.norm()) / cam_axis.dot(e0_axis / e0_axis.norm());
						p_inters_e0 = cam_axis * d_origin_e0;
					}
					if (cam_axis.dot(e1_axis / e1_axis.norm()) != 0.0) {
						d_origin_e1 = p_face_e1.dot(e1_axis / e1_axis.norm()) / cam_axis.dot(e1_axis / e1_axis.norm());
						p_inters_e1 = cam_axis * d_origin_e1;
					}
					if (cam_axis.dot(z_axis / z_axis.norm()) != 0.0) {
						d_origin_ez = p_face_ez.dot(z_axis / z_axis.norm()) / cam_axis.dot(z_axis / z_axis.norm());
						p_inters_ez = cam_axis * d_origin_ez;
					}

					// Calculate the distance to each face center
					vecon_e0 = p_inters_e0 - p_face_e0;
					vecon_e1 = p_inters_e1 - p_face_e1;
					vecon_ez = p_inters_ez - p_face_ez;

					// Assess the following three conditions
					// 1. Calculate the projection vector to the 'inters' (intersection) point on each face
					// // face e0
					proj_e0_e1 = std::abs(vecon_e0.dot(e1_axis) / e1_axis.norm());
					proj_e0_ez = std::abs(vecon_e0.dot(z_axis) / z_axis.norm());
					std::cout << "proj_e0: " << proj_e0_e1 << "(" << e1_axis.norm() << ")" << " " << proj_e0_ez << "(" << z_axis.norm() << ")" << std::endl;
					if (!found && proj_e0_e1 <= e1_axis.norm() && proj_e0_ez <= z_axis.norm()) {
						if (e1_axis.norm() >= z_axis.norm()) {
							axis = e1_axis;
						}
						else {
							axis = z_axis;
						}
						found = true;
					}
					else {
						if (proj_e0_e1 - e1_axis.norm() > 0 && proj_e0_ez - z_axis.norm() <= 0) {
							d_face_e0 = proj_e0_e1 - e1_axis.norm();
						}
						else if (proj_e0_ez - z_axis.norm() > 0 && proj_e0_e1 - e1_axis.norm() <= 0) {
							d_face_e0 = proj_e0_ez - z_axis.norm();
						}
						else if (proj_e0_e1 - e1_axis.norm() > 0 && proj_e0_ez - z_axis.norm() > 0) {
							if (proj_e0_e1 - e1_axis.norm() <= proj_e0_ez - z_axis.norm()) {
								d_face_e0 = proj_e0_e1 - e1_axis.norm();
							}
							else {
								d_face_e0 = proj_e0_ez - z_axis.norm();
							}
						}
					}
					// // face e1
					proj_e1_e0 = std::abs(vecon_e1.dot(e0_axis) / e0_axis.norm());
					proj_e1_ez = std::abs(vecon_e1.dot(z_axis) / z_axis.norm());
					std::cout << "proj_e1: " << proj_e1_e0 << "(" << e0_axis.norm() << ")" << " " << proj_e1_ez << "(" << z_axis.norm() << ")" << std::endl;
					if (!found && proj_e1_e0 <= e0_axis.norm() && proj_e1_ez <= z_axis.norm()) {
						if (e0_axis.norm() >= z_axis.norm()) {
							axis = e0_axis;
						}
						else {
							axis = z_axis;
						}
						found = true;
					}
					else {
						if (proj_e1_e0 - e0_axis.norm() > 0 && proj_e1_ez - z_axis.norm() <= 0) {
							d_face_e1 = proj_e1_e0 - e0_axis.norm();
						}
						else if (proj_e1_ez - z_axis.norm() > 0 && proj_e1_e0 - e0_axis.norm() <= 0) {
							d_face_e1 = proj_e1_ez - z_axis.norm();
						}
						else if (proj_e1_e0 - e0_axis.norm() > 0 && proj_e1_ez - z_axis.norm() > 0) {
							if (proj_e1_e0 - e0_axis.norm() <= proj_e1_ez - z_axis.norm()) {
								d_face_e1 = proj_e1_e0 - e0_axis.norm();
							}
							else {
								d_face_e1 = proj_e1_ez - z_axis.norm();
							}
						}
					}
					// // face ez
					proj_ez_e0 = std::abs(vecon_ez.dot(e0_axis) / e0_axis.norm());
					proj_ez_e1 = std::abs(vecon_ez.dot(e1_axis) / e1_axis.norm());
					std::cout << "proj_ez: " << proj_ez_e0 << "(" << e0_axis.norm() << ")" << " " << proj_ez_e1 << "(" << e1_axis.norm() << ")" << std::endl;
					if (!found && proj_ez_e0 <= e0_axis.norm() && proj_ez_e1 <= e1_axis.norm()) {
						if (e0_axis.norm() >= e1_axis.norm()) {
							axis = e0_axis;
						}
						else {
							axis = e1_axis;
						}
						found = true;
					}
					else {
						if (proj_ez_e0 - e0_axis.norm() > 0 && proj_ez_e1 - e1_axis.norm() <= 0) {
							d_face_ez = proj_ez_e0 - e0_axis.norm();
						}
						else if (proj_ez_e1 - e1_axis.norm() > 0 && proj_ez_e0 - e0_axis.norm() <= 0) {
							d_face_ez = proj_ez_e1 - e1_axis.norm();
						}
						else if (proj_ez_e0 - e0_axis.norm() > 0 && proj_ez_e1 - e1_axis.norm() > 0) {
							if (proj_ez_e0 - e0_axis.norm() <= proj_ez_e1 - e1_axis.norm()) {
								d_face_ez = proj_ez_e0 - e0_axis.norm();
							}
							else {
								d_face_ez = proj_ez_e1 - e1_axis.norm();
							}
						}
					}

					// 2. Select the face based on the shortest distance 'd_face'
					if (!found) {
						// // face e0
						if (p_inters_e0.z() > 0.1 && d_face_e0 <= d_face_e1 && d_face_e0 <= d_face_ez) {
							if (e1_axis.norm() >= z_axis.norm()) {
								axis = e1_axis;
							}
							else {
								axis = z_axis;
							}
						}
						// // face e1
						if (p_inters_e1.z() > 0.1 && d_face_e1 <= d_face_e0 && d_face_e1 <= d_face_ez) {
							if (e0_axis.norm() >= z_axis.norm()) {
								axis = e0_axis;
							}
							else {
								axis = z_axis;
							}
						}
						// // face ez
						if (p_inters_ez.z() > 0.1 && d_face_ez <= d_face_e0 && d_face_ez <= d_face_e1) {
							if (e0_axis.norm() >= e1_axis.norm()) {
								axis = e0_axis;
							}
							else {
								axis = e1_axis;
							}
						}
					}

					//// Choose the appropriate grasp_type
					if (2.0 * axis.norm() < length_threshold) {
						if (target_grasp_size_.value < grasp_size_threshold) {
							grasp_type = robin::hand::GRASP::LATERAL;
						}
						else {
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
						}
						else {							
							grasp_type = robin::hand::GRASP::PALMAR;
						}
					}
					else {
						grasp_type = robin::hand::GRASP::PALMAR;
					}
					break;
				}
			}


			target_grasp_type_.buffer.push_back(int(grasp_type));						
			std::cout << "Estimated grasp_type: ";
			switch (int(grasp_type)) {
			case int(robin::hand::GRASP::PALMAR):
				std::cout << "PALMAR";
				break;
			case int(robin::hand::GRASP::LATERAL):
				std::cout << "LATERAL";
				break;
			}

			target_grasp_type_.value = target_grasp_type_.buffer.back(); // Direct assignement without var.update()
			std::cout << " with majvote: ";
			switch (int(target_grasp_type_.value)) {
			case int(robin::hand::GRASP::PALMAR) :
				std::cout << "PALMAR" << std::endl;
				break;
			case int(robin::hand::GRASP::LATERAL) :
				std::cout << "LATERAL" << std::endl;
				break;
			}
		}

		void ControlContinuous::estimate_tilt_angle(robin::Primitive3* prim)
		{						
			// Tilt angle calculated as an absolute supination angle, i.e. measured from the full pronated wrist position.
			float tilt_angle(target_tilt_angle_.value);

			if (!prim->isEmpty())
			{
				Primitive3Type prim3_type = find_primitive3_type(prim);
				if (prim3_type == Primitive3Type::PRIMITIVE3_SPHERE)
				{
					tilt_angle = 0.0; // Palm facing down.
				}
				else if (prim3_type == Primitive3Type::PRIMITIVE3_CUBOID || prim3_type == Primitive3Type::PRIMITIVE3_CYLINDER)
				{
					// Angle of the projection of the Primitive3 axis into the xOy plane of the camera [-Pi,+Pi].
					float projected_angle(std::atan2(prim->getProperty_axis_y(), prim->getProperty_axis_x()));

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

					// Shifting the tilt angle of the Primitive3
					switch (int(target_grasp_type_.value))
					{
						case int(robin::hand::GRASP::PALMAR) :
							// Do nothing else.
							break;
						case int(robin::hand::GRASP::LATERAL) :
							// The tilt angle shall be shifted by Pi/2 or -Pi/2.
							if (hand_->isRightHand()) {
								// Right-hand prosthesis (positive angle)
								if (tilt_angle <= M_PI / 4) { tilt_angle += M_PI / 2; }
								else { tilt_angle -= M_PI / 2; }
							}
							else {
								// Left-hand prosthesis (negative angle)
								if (tilt_angle <= -M_PI / 4) { tilt_angle += M_PI / 2; }
								else { tilt_angle -= M_PI / 2; }
							}
						break;
					}
				}
			}

			target_tilt_angle_.buffer.push_back(tilt_angle);
			std::cout << "Estimated tilt_angle: " << tilt_angle;

			target_tilt_angle_.update(filter_, window_size_);
			std::cout << " with movavg: " << target_tilt_angle_.value << std::endl;
		}





		void ControlContinuous::setDataManager(robin::data::DataManager& dm)
		{
			dm_ = &dm;
		}

		int ControlContinuous::saveEvent(bool flag) const
		{
			robin::data::EventData data;
			data.mode = int(state_auto_); // Mode Auto[1]/Manual[0]
			data.rotate = int(state_myomenu_); // Rotation
			data.grasp_type = int(target_grasp_type_.value);
			data.grasp_size_estim = target_grasp_size_.value; // grasp_size from the CVsolver
			data.grasp_size_slack = hand_->getGraspSize(); // grasp_size with slack/tolerance
			data.tilt_angle_estim = target_tilt_angle_.value; // tilt_angle from the CVsolver
			data.tilt_angle_slack = hand_->getWristSupProAngle(); // tilt_angle with slack/tolerance
			data.flag = flag;
			if (!hand_->isRightHand()) { data.tilt_angle_slack *= -1; }
			
			if (dm_->saveEvent(data) == 0)
				return 0;
			else
				return -1;
		}




		bool ControlContinuous::isKeyPressed(char* c)
		{
			if (kbhit()) {
				*c = getch();
				return true;
			} else {
				return false;
			}
		}
	}
}