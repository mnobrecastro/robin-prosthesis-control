#include "primitive3_cuboid.h"

//#define RENDER

namespace robin
{
	Primitive3Cuboid::Primitive3Cuboid()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = cloud;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients_ = coefficients;
		// Initialization of the 10-element cube coefficients
		// {Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth}
		coefficients_->values.push_back(0.0); //0
		coefficients_->values.push_back(0.0); //1
		coefficients_->values.push_back(0.0); //2
		coefficients_->values.push_back(0.0); //3
		coefficients_->values.push_back(0.0); //4
		coefficients_->values.push_back(0.0); //5
		coefficients_->values.push_back(0.0); //6
		coefficients_->values.push_back(0.0); //7
		coefficients_->values.push_back(0.0); //8
		coefficients_->values.push_back(0.0); //9

		// Initialization of intersection point
		inters_point_ = Eigen::Vector3f(0.0, 0.0, 0.0);

		isempty_ = true;
	}
	Primitive3Cuboid::~Primitive3Cuboid() {}

	void Primitive3Cuboid::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
	{
		//if (visualizeOnOff_)
		//viewer->addCube(*coefficients_, "cube");
		for (auto plane : planes_) {
			plane->setVisualizeOnOff(false);
			plane->visualize(viewer);
		}

		pcl::PointXYZ ref_center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ ref_point_x(properties_.center_x + properties_.e0_x, properties_.center_y + properties_.e0_y, properties_.center_z + properties_.e0_z);
		pcl::PointXYZ ref_point_y(properties_.center_x + properties_.e1_x, properties_.center_y + properties_.e1_y, properties_.center_z + properties_.e1_z);
		pcl::PointXYZ ref_point_z(properties_.center_x + properties_.axis_x, properties_.center_y + properties_.axis_y, properties_.center_z + properties_.axis_z);
		viewer->addLine<pcl::PointXYZ>(ref_center, ref_point_x, 1.0, 0.4, 0.4, "axis0");
		viewer->addLine<pcl::PointXYZ>(ref_center, ref_point_y, 0.4, 1.0, 0.4, "axis1");
		viewer->addLine<pcl::PointXYZ>(ref_center, ref_point_z, 0.4, 0.4, 1.0, "axis2");

		if (visualizeOnOff_) {

			// Intersection point (as a sphere)
			pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
			coefs->values.push_back(inters_point_.x());
			coefs->values.push_back(inters_point_.y());
			coefs->values.push_back(inters_point_.z());
			coefs->values.push_back(0.005);
			viewer->addSphere(*coefs, "inters_sph");


			viewer->addCube(*coefficients_, "cube");
			viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 76.0 / 255.0, 0 / 255.0, 102 / 255.0, "cube"); //153,0,204
			if (false) {
				//if (view_face_idx_ == -1) {
				viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, "cube");
			}
			else {
				viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.1, "cube");

				// Automatic selection of the Cuboid face
				size_t view_face_idx;

				// ----
				float projection(0.0), proj_e0_e1(0.0), proj_e0_ez(0.0), proj_e1_e0(0.0), proj_e1_ez(0.0), proj_ez_e0(0.0), proj_ez_e1(0.0);
				float d_origin_e0(0.0), d_origin_e1(0.0), d_origin_ez(0.0), d_face_e0(0.0), d_face_e1(0.0), d_face_ez(0.0);
				Eigen::Vector3f axis, cam_axis(0.0, 0.0, 1.0), e0_axis, e1_axis, z_axis, p_center;
				Eigen::Vector3f p_face_e0, p_face_e1, p_face_ez, p_inters_e0(0.0, 0.0, 0.0), p_inters_e1(0.0, 0.0, 0.0), p_inters_ez(0.0, 0.0, 0.0), vecon_e0(0.0, 0.0, 0.0), vecon_e1(0.0, 0.0, 0.0), vecon_ez(0.0, 0.0, 0.0);
				bool found(false);

				//// Ray casting
				p_center = { properties_.center_x,properties_.center_y, properties_.center_z };
				e0_axis = { properties_.e0_x, properties_.e0_y, properties_.e0_z };
				e1_axis = { properties_.e1_x, properties_.e1_y, properties_.e1_z };
				z_axis = { properties_.axis_x, properties_.axis_y, properties_.axis_z };

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

					found = true;
					//this->setIntersectionPoint(p_inters_e0);
					//this->setFaceHighlight(0);
					view_face_idx = 0;
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

					found = true;
					//this->setIntersectionPoint(p_inters_e1);
					//this->setFaceHighlight(1);
					view_face_idx = 1;
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

					found = true;
					//this->setIntersectionPoint(p_inters_ez);
					//this->setFaceHighlight(2);
					view_face_idx = 2;
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
						//this->setIntersectionPoint(p_inters_e0);
						//this->setFaceHighlight(0);
						view_face_idx = 0;
					}
					// // face e1
					if (p_inters_e1.z() > 0.1 && d_face_e1 <= d_face_e0 && d_face_e1 <= d_face_ez) {
						//this->setIntersectionPoint(p_inters_e1);
						//this->setFaceHighlight(1);
						view_face_idx = 1;
					}
					// // face ez
					if (p_inters_ez.z() > 0.1 && d_face_ez <= d_face_e0 && d_face_ez <= d_face_e1) {
						//this->setIntersectionPoint(p_inters_ez);
						//this->setFaceHighlight(2);
						view_face_idx = 2;
					}
				}
				// ----


				for (int idx(0); idx < 3; ++idx) {

					if (idx < planes_.size()) {
						planes_[idx]->setVisualizeOnOff(false);
						planes_[idx]->visualize(viewer);
					}

					if (idx == view_face_idx) {
						//Render the selected cuboid face as a cube object with small thickness

						pcl::ModelCoefficients::Ptr coefs_cube(new pcl::ModelCoefficients);
						// Initialization of the 10-element cube coefficients
						// {Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth}
						coefs_cube->values.push_back(0.0); //0
						coefs_cube->values.push_back(0.0); //1
						coefs_cube->values.push_back(0.0); //2
						coefs_cube->values.push_back(0.0); //3
						coefs_cube->values.push_back(0.0); //4
						coefs_cube->values.push_back(0.0); //5
						coefs_cube->values.push_back(0.0); //6
						coefs_cube->values.push_back(0.0); //7
						coefs_cube->values.push_back(0.0); //8
						coefs_cube->values.push_back(0.0); //9

						float width(0.0), height(0.0), depth(0.0);
						Eigen::Vector3f e0_axis, e1_axis, z_axis;
						Eigen::Matrix3f mori;
						Eigen::Quaternionf quat;
						Eigen::Vector3f face_center, cube_center;

						/*
						 *    ___________
						 *   |   Z ^    ||
						 *   |     |    ||
						 *   |     |    ||
						 *   |     ---> ||
						 *   |    /     ||
						 *   |  e0	    ||
						 *   |__________/
						 */

						width = properties_.width; // In the e1-direction
						height = properties_.height; // In the z-direction
						depth = properties_.depth; // In the e0-direction

						e1_axis = { properties_.e1_x, properties_.e1_y, properties_.e1_z };
						z_axis = { properties_.axis_x, properties_.axis_y, properties_.axis_z };
						e0_axis = { properties_.e0_x, properties_.e0_y, properties_.e0_z };

						switch (view_face_idx) {

						case 0:
							face_center(0) = properties_.center_x + e0_axis(0);
							face_center(1) = properties_.center_y + e0_axis(1);
							face_center(2) = properties_.center_z + e0_axis(2);

							// Transformation matrix
							e0_axis.normalize();
							mori(0, 0) = e0_axis(0);
							mori(1, 0) = e0_axis(1);
							mori(2, 0) = e0_axis(2);
							e1_axis.normalize();
							mori(0, 1) = e1_axis(0);
							mori(1, 1) = e1_axis(1);
							mori(2, 1) = e1_axis(2);
							z_axis.normalize();
							mori(0, 2) = z_axis(0);
							mori(1, 2) = z_axis(1);
							mori(2, 2) = z_axis(2);

							quat = Eigen::Quaternionf(mori);

							//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
							coefs_cube->values[0] = face_center.x(); //Tx
							coefs_cube->values[1] = face_center.y(); //Ty
							coefs_cube->values[2] = face_center.z(); //Tz
							coefs_cube->values[3] = quat.x(); //Qx
							coefs_cube->values[4] = quat.y(); //Qy
							coefs_cube->values[5] = quat.z(); //Qz
							coefs_cube->values[6] = quat.w(); //Qw
							coefs_cube->values[7] = 0.001; //width;
							coefs_cube->values[8] = height;
							coefs_cube->values[9] = depth;

							break;
						case 1:
							if (e1_axis(2) > 0) {
								face_center(0) = properties_.center_x - e1_axis(0);
								face_center(1) = properties_.center_y - e1_axis(1);
								face_center(2) = properties_.center_z - e1_axis(2);
							}
							else {
								face_center(0) = properties_.center_x + e1_axis(0);
								face_center(1) = properties_.center_y + e1_axis(1);
								face_center(2) = properties_.center_z + e1_axis(2);
							}

							// Transformation matrix					
							e1_axis.normalize();
							mori(0, 0) = e1_axis(0);
							mori(1, 0) = e1_axis(1);
							mori(2, 0) = e1_axis(2);
							z_axis.normalize();
							mori(0, 1) = z_axis(0);
							mori(1, 1) = z_axis(1);
							mori(2, 1) = z_axis(2);
							e0_axis.normalize();
							mori(0, 2) = e0_axis(0);
							mori(1, 2) = e0_axis(1);
							mori(2, 2) = e0_axis(2);

							quat = Eigen::Quaternionf(mori);

							//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
							coefs_cube->values[0] = face_center.x(); //Tx
							coefs_cube->values[1] = face_center.y(); //Ty
							coefs_cube->values[2] = face_center.z(); //Tz
							coefs_cube->values[3] = quat.x(); //Qx
							coefs_cube->values[4] = quat.y(); //Qy
							coefs_cube->values[5] = quat.z(); //Qz
							coefs_cube->values[6] = quat.w(); //Qw
							coefs_cube->values[7] = 0.001; //width;
							coefs_cube->values[8] = depth; //height;
							coefs_cube->values[9] = width; //depth;

							break;
						case 2:
							if (z_axis(1) > 0) {
								face_center(0) = properties_.center_x - z_axis(0);
								face_center(1) = properties_.center_y - z_axis(1);
								face_center(2) = properties_.center_z - z_axis(2);
							}
							else {
								face_center(0) = properties_.center_x + z_axis(0);
								face_center(1) = properties_.center_y + z_axis(1);
								face_center(2) = properties_.center_z + z_axis(2);
							}

							// Transformation matrix
							z_axis.normalize();
							mori(0, 0) = z_axis(0);
							mori(1, 0) = z_axis(1);
							mori(2, 0) = z_axis(2);
							e0_axis.normalize();
							mori(0, 1) = e0_axis(0);
							mori(1, 1) = e0_axis(1);
							mori(2, 1) = e0_axis(2);
							e1_axis.normalize();
							mori(0, 2) = e1_axis(0);
							mori(1, 2) = e1_axis(1);
							mori(2, 2) = e1_axis(2);

							quat = Eigen::Quaternionf(mori);

							//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
							coefs_cube->values[0] = face_center.x(); //Tx
							coefs_cube->values[1] = face_center.y(); //Ty
							coefs_cube->values[2] = face_center.z(); //Tz
							coefs_cube->values[3] = quat.x(); //Qx
							coefs_cube->values[4] = quat.y(); //Qy
							coefs_cube->values[5] = quat.z(); //Qz
							coefs_cube->values[6] = quat.w(); //Qw
							coefs_cube->values[7] = 0.001; //width;
							coefs_cube->values[8] = width; //height;
							coefs_cube->values[9] = height; //depth;

							break;
						default:
							// None
							break;
						}
						viewer->addCube(*coefs_cube, "plane");
						viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 76.0 / 255.0, 0 / 255.0, 102 / 255.0, "plane"); //153,0,204
					}
				}
			}
		}

		pcl::PointXYZ center(properties_.center_x, properties_.center_y, properties_.center_z);
		pcl::PointXYZ point_x(properties_.center_x + properties_.e0_x, properties_.center_y + properties_.e0_y, properties_.center_z + properties_.e0_z);
		pcl::PointXYZ point_y(properties_.center_x + properties_.e1_x, properties_.center_y + properties_.e1_y, properties_.center_z + properties_.e1_z);
		pcl::PointXYZ point_z(properties_.center_x + properties_.axis_x, properties_.center_y + properties_.axis_y, properties_.center_z + properties_.axis_z);
		viewer->addLine<pcl::PointXYZ>(center, point_x, 1.0, 0.4, 0.4, "axis0");
		viewer->addLine<pcl::PointXYZ>(center, point_y, 0.4, 1.0, 0.4, "axis1");
		viewer->addLine<pcl::PointXYZ>(center, point_z, 0.4, 0.4, 1.0, "axis2");

		// Wireframe cube
		//pcl::PointXYZ pFUL(plot_[0] - 0.005, plot_[1] - 0.005, plot_[2] - 0.005);
		//pcl::PointXYZ pFUR(plot_[0] + 0.005, plot_[1] - 0.005, plot_[2] - 0.005);
		//pcl::PointXYZ pFDL(plot_[0] - 0.005, plot_[1] + 0.005, plot_[2] - 0.005);
		//pcl::PointXYZ pFDR(plot_[0] + 0.005, plot_[1] + 0.005, plot_[2] - 0.005);

		//pcl::PointXYZ pBUL(plot_[0] - 0.005, plot_[1] - 0.005, plot_[2] + 0.005);
		//pcl::PointXYZ pBUR(plot_[0] + 0.005, plot_[1] - 0.005, plot_[2] + 0.005);
		//pcl::PointXYZ pBDL(plot_[0] - 0.005, plot_[1] + 0.005, plot_[2] + 0.005);
		//pcl::PointXYZ pBDR(plot_[0] + 0.005, plot_[1] + 0.005, plot_[2] + 0.005);

		//viewer->addLine<pcl::PointXYZ>(pFUL, pFUR, 255, 255, 255, "cFU");
		//viewer->addLine<pcl::PointXYZ>(pFUR, pFDR, 255, 255, 255, "cFR");
		//viewer->addLine<pcl::PointXYZ>(pFDR, pFDL, 255, 255, 255, "cFD");
		//viewer->addLine<pcl::PointXYZ>(pFDL, pFUL, 255, 255, 255, "cFL");

		//viewer->addLine<pcl::PointXYZ>(pBUL, pBUR, 255, 255, 255, "cBU");
		//viewer->addLine<pcl::PointXYZ>(pBUR, pBDR, 255, 255, 255, "cBR");
		//viewer->addLine<pcl::PointXYZ>(pBDR, pBDL, 255, 255, 255, "cBD");
		//viewer->addLine<pcl::PointXYZ>(pBDL, pBUL, 255, 255, 255, "cBL");

		//viewer->addLine<pcl::PointXYZ>(pBUL, pFUL, 255, 255, 255, "cLU");
		//viewer->addLine<pcl::PointXYZ>(pBDL, pFDL, 255, 255, 255, "cLD");
		//viewer->addLine<pcl::PointXYZ>(pBUR, pFUR, 255, 255, 255, "cRU");
		//viewer->addLine<pcl::PointXYZ>(pBDR, pFDR, 255, 255, 255, "cRD");

		// Intersection point (as a sphere)
		pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
		coefs->values.push_back(inters_point_.x());
		coefs->values.push_back(inters_point_.y());
		coefs->values.push_back(inters_point_.z());
		coefs->values.push_back(0.005);
		viewer->addSphere(*coefs, "inters_sph");
	}

	void Primitive3Cuboid::reset()
	{
		cloud_->points.clear();
		coefficients_->values[0] = -0.001;
		coefficients_->values[1] = -0.001;
		coefficients_->values[2] = -0.001;
		coefficients_->values[3] = 0.000;
		coefficients_->values[4] = 0.000;
		coefficients_->values[5] = 0.000;
		coefficients_->values[6] = 0.000;
		coefficients_->values[7] = 0.000;
		coefficients_->values[8] = 0.000;
		coefficients_->values[9] = 0.000;

		isempty_ = true;
	}

	void Primitive3Cuboid::setCoefficients(std::vector<float> v)
	{
		if (v.size() == 10) {
			coefficients_->values[0] = v[0];
			coefficients_->values[1] = v[1];
			coefficients_->values[2] = v[2];
			coefficients_->values[3] = v[3];
			coefficients_->values[4] = v[4];
			coefficients_->values[5] = v[5];
			coefficients_->values[6] = v[6];
			coefficients_->values[7] = v[7];
			coefficients_->values[8] = v[8];
			coefficients_->values[9] = v[9];

			isempty_ = false;
		}
	}




	void Primitive3Cuboid::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool normals)
	{
		size_t n_planes(0); int n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_planes < 3) {
			// A minimum of 2 planes are enough to fully define the Primitive3Cuboid.
			if (n_planes == 0) {
				Primitive3Plane* plane(new Primitive3Plane());
				plane->fit(cloud, normals);
				*cloud_ += *(plane->getPointCloud());
				planes_.push_back(plane);
			}
			else {
				Eigen::Vector3f normal_axis(planes_[0]->getProperty_axis_x(), planes_[0]->getProperty_axis_y(), planes_[0]->getProperty_axis_z());
				//Primitive3Plane* plane(new Primitive3Plane(robin::PLANE_TYPE::PERPENDICULAR, normal_axis, 15.0));
				Primitive3Plane* plane(new Primitive3Plane());
				try {
					plane->fit(cloud, normals);
				}
				catch (...) {}
				// Avoid cases where PERPENDICULAR planes were not found
				Eigen::Vector3f new_axis(plane->getProperty_axis_x(), plane->getProperty_axis_y(), plane->getProperty_axis_z());
				float angle(std::acos(normal_axis.dot(new_axis)/(normal_axis.norm() * new_axis.norm())));
				if (!plane->getPointCloud()->points.empty() && std::abs(angle-M_PI/2) < 5 * M_PI/180.0) {
					*cloud_ += *(plane->getPointCloud());
					planes_.push_back(plane);
				}
			}
			++n_planes;
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	void Primitive3Cuboid::fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		if (!cloud_->points.empty()) {
			cloud_->clear();
		}
		if (!planes_.empty()) {
			for (auto plane : planes_) {
				delete plane;
			}
			planes_.clear();
		}

		size_t n_planes(0); int n_pts(cloud->points.size());
		while (cloud->points.size() > 0.1 * n_pts && n_planes < 3) { 
			// A minimum of 2 planes are enough to fully define the Primitive3Cuboid.
			if (n_planes == 0) {
				Primitive3Plane* plane(new Primitive3Plane());
				plane->fit(cloud, seg);
				*cloud_ += *(plane->getPointCloud());
				planes_.push_back(plane);
				++n_planes;
			}
			else {
				Eigen::Vector3f normal_axis(planes_[0]->getProperty_axis_x(), planes_[0]->getProperty_axis_y(), planes_[0]->getProperty_axis_z());				
				//Primitive3Plane* plane(new Primitive3Plane(robin::PLANE_TYPE::PERPENDICULAR, normal_axis, 1.0)); //15.0
				Primitive3Plane* plane(new Primitive3Plane());
				try {
					plane->fit(cloud, seg);
				}
				catch (...) {}
				// Avoid cases where PERPENDICULAR planes were not found
				Eigen::Vector3f new_axis(plane->getProperty_axis_x(), plane->getProperty_axis_y(), plane->getProperty_axis_z());
				float angle(std::acos(normal_axis.dot(new_axis) / (normal_axis.norm() * new_axis.norm())));
				if (!plane->getPointCloud()->points.empty() && std::abs(angle - M_PI / 2) < 5 * M_PI / 180.0) {
					*cloud_ += *(plane->getPointCloud());
					planes_.push_back(plane);
					++n_planes;
				}
			}					
		}

		/* 1. Validate the fit. */
		if (this->is_fit_valid()) {
			/* 2. Correct model coeffficients. */
			this->correct_coefficients();
		}
		/* 3. Update object properties. */
		this->update_properties();
	}

	/* Checks if the fit is valid. */
	bool Primitive3Cuboid::is_fit_valid()
	{
		/* Checks if all planes are valid. */
		if (!(planes_.size() > 0)) {
			this->reset();
			return false;
		}

		/* Checks if the cut sub-primitives are valid. */
		bool valid_subprims(true);
		if (!subprims_.empty()) {
			for (auto arr : subprims_) {
				for (auto sp : arr) {
					if (sp->getPointCloud()->empty()) {
						this->reset();
						return false;
					}
				}
			}
		}

		isempty_ = false;
		return true;
	}



	/* Correct the obtained coefficients if necessary. */
	void Primitive3Cuboid::correct_coefficients()
	{
		if (planes_.size() > 0) {
			float width(0.0), height(0.0), depth(0.001);
			Eigen::Vector3f e0_axis, e1_axis, z_axis;
			Eigen::Matrix3f mori;
			Eigen::Vector3f face_center, cube_center;
			Eigen::Vector3f intersection_axis, point_on_axis;

			if (planes_.size() == 1) {
				/*
				 *    ___________
				 *   |   Z ^    ||
				 *   |     |    ||
				 *   |     |    ||
				 *   |     ---> ||
				 *   |    /     ||
				 *   |  e0	    ||
				 *   |__________/
				 */

				width = planes_[0]->getProperty_width(); // In the e1-direction
				height = planes_[0]->getProperty_height(); // In the z-direction

				// Plane z -> Cube e0
				e0_axis = { planes_[0]->getProperty_axis_x(), planes_[0]->getProperty_axis_y(), planes_[0]->getProperty_axis_z() };
				e0_axis.normalize();
				e0_axis *= depth / 2;
				// Plane -e1 -> Cube e1
				e1_axis = { -planes_[0]->getProperty_e1_x(), -planes_[0]->getProperty_e1_y(), -planes_[0]->getProperty_e1_z() };
				e1_axis.normalize();
				e1_axis *= width / 2;
				// Plane e0 -> Cube z
				z_axis = { planes_[0]->getProperty_e0_x(), planes_[0]->getProperty_e0_y(), planes_[0]->getProperty_e0_z() };
				z_axis.normalize();
				z_axis *= height / 2;
			}
			else if (planes_.size() > 1) {
				// Case of 2 or 3
				// e0 is by convention the face closest to the view
				// Z defines HEIGHT, e0 defines DEPTH, e1 defines WIDTH
				/*
				 *      ___________
				 *     /   Z      /|
				 *    /    ^     / |
				 *   /_____|____/F1|
				 *   |F0   |    |  |
				 *   |     |    |  |
				 *   |     |    |  |
				 *   |     -----|->|
				 *   |    /     |e1|
				 *   |   /	    |  |
				 *   | e0       | /
				 *   |__________|/
				 */

				float a1(planes_[0]->getProperty_axis_x());
				float b1(planes_[0]->getProperty_axis_y());
				float c1(planes_[0]->getProperty_axis_z());
				float d1(planes_[0]->getProperty_d());

				float a2(planes_[1]->getProperty_axis_x());
				float b2(planes_[1]->getProperty_axis_y());
				float c2(planes_[1]->getProperty_axis_z());
				float d2(planes_[1]->getProperty_d());

				// Calculate the intersection axis between the two planes (defines the new Z)
				intersection_axis(0) = b1 / a1 * (c2 - a2 * c1 / a1) / (b2 - a2 * b1 / a1) - c1 / a1;
				intersection_axis(1) = -(c2 - a2 * c1 / a1) / (b2 - a2 * b1 / a1);
				intersection_axis(2) = 1;

				// Calculate a point on the axis
				point_on_axis(0) = intersection_axis(0) + b1 / a1 * (d2 - a2 * d1 / a1) / (b2 - a2 * b1 / a1) - d1 / a1;
				point_on_axis(1) = intersection_axis(1) - (d2 - a2 * d1 / a1) / (b2 - a2 * b1 / a1);
				point_on_axis(2) = 0;

				// Calculate height from projecting all points from all planes
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for (auto plane : planes_) {
					*cloud += *plane->getPointCloud();
				}
				std::array<float, 2> height_arr(getPointCloudExtremes(
					*cloud,
					pcl::PointXYZ(point_on_axis.x(), point_on_axis.y(), point_on_axis.z()),
					pcl::PointXYZ(intersection_axis.x(), intersection_axis.y(), intersection_axis.z())
				));
				height = height_arr[1] - height_arr[0];

				// Calculate width
				Eigen::Vector3f face0_e0_axis = { planes_[0]->getProperty_axis_x(), planes_[0]->getProperty_axis_y(), planes_[0]->getProperty_axis_z() }; // normalized
				Eigen::Vector3f face0_e1_axis = intersection_axis.cross(face0_e0_axis); // new/updated face0_e1_axis
				std::array<float, 2> width_arr(getPointCloudExtremes(
					*planes_[0]->getPointCloud(),
					pcl::PointXYZ(planes_[0]->getProperty_center_x(), planes_[0]->getProperty_center_y(), planes_[0]->getProperty_center_z()),
					pcl::PointXYZ(face0_e1_axis.x(), face0_e1_axis.y(), face0_e1_axis.z()) //intersection_axis.
				));
				width = width_arr[1] - width_arr[0];

				// Calculate depth
				Eigen::Vector3f face1_e0_axis = { planes_[1]->getProperty_axis_x(), planes_[1]->getProperty_axis_y(), planes_[1]->getProperty_axis_z() }; // normalized (no longer used)
				Eigen::Vector3f face1_e1_axis = intersection_axis.cross(face1_e0_axis); // new/updated face1_e1_axis (no longer used)
				std::array<float, 2> depth_arr(getPointCloudExtremes(
					*planes_[1]->getPointCloud(),
					pcl::PointXYZ(planes_[0]->getProperty_center_x(), planes_[0]->getProperty_center_y(), planes_[0]->getProperty_center_z()), //planes_[1]
					pcl::PointXYZ(face0_e0_axis.x(), face0_e0_axis.y(), face0_e0_axis.z()) //intersection_axis.
				));
				depth = depth_arr[1] - depth_arr[0];

				// Plane z -> Cube e0
				e0_axis = face0_e0_axis;
				e0_axis.normalize();
				e0_axis *= depth / 2;
				// Plane -e1 -> Cube e1
				e1_axis = face0_e1_axis;
				e1_axis.normalize();
				e1_axis *= width / 2;
				// Plane e0 -> Cube z
				z_axis = intersection_axis;
				z_axis.normalize();
				z_axis *= height / 2;
			}

			face_center(0) = planes_[0]->getProperty_center_x();
			face_center(1) = planes_[0]->getProperty_center_y();
			face_center(2) = planes_[0]->getProperty_center_z();

			cube_center(0) = face_center.x() - e0_axis(0);
			cube_center(1) = face_center.y() - e0_axis(1);
			cube_center(2) = face_center.z() - e0_axis(2);

			// Since width (plane e0) <= height (plane e1)
			properties_.e0_x = e0_axis(0);
			properties_.e0_y = e0_axis(1);
			properties_.e0_z = e0_axis(2);
			properties_.e1_x = e1_axis(0);
			properties_.e1_y = e1_axis(1);
			properties_.e1_z = e1_axis(2);
			properties_.axis_x = z_axis(0);
			properties_.axis_y = z_axis(1);
			properties_.axis_z = z_axis(2);

			// Transformation matrix
			e0_axis.normalize();
			mori(0, 0) = e0_axis(0);
			mori(1, 0) = e0_axis(1);
			mori(2, 0) = e0_axis(2);
			e1_axis.normalize();
			mori(0, 1) = e1_axis(0);
			mori(1, 1) = e1_axis(1);
			mori(2, 1) = e1_axis(2);
			z_axis.normalize();
			mori(0, 2) = z_axis(0);
			mori(1, 2) = z_axis(1);
			mori(2, 2) = z_axis(2);

			Eigen::Quaternionf quat(mori);

			//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
			coefficients_->values[0] = cube_center.x(); //Tx
			coefficients_->values[1] = cube_center.y(); //Ty
			coefficients_->values[2] = cube_center.z(); //Tz
			coefficients_->values[3] = quat.x(); //Qx
			coefficients_->values[4] = quat.y(); //Qy
			coefficients_->values[5] = quat.z(); //Qz
			coefficients_->values[6] = quat.w(); //Qw
			coefficients_->values[7] = depth; //width;
			coefficients_->values[8] = width; //height;
			coefficients_->values[9] = height; //depth;
		}
	}

	/* Update the properties of the Primitive3. */
	void Primitive3Cuboid::update_properties()
	{
		properties_.center_x = coefficients_->values[0];
		properties_.center_y = coefficients_->values[1];
		properties_.center_z = coefficients_->values[2];
		properties_.width = coefficients_->values[7];
		properties_.height = coefficients_->values[8];
		properties_.depth = coefficients_->values[9];

		std::cout << "Primitive3Cuboid properties were updated." << std::endl;
		std::cout << "\t" << properties_.width << " " << properties_.height << " " << properties_.depth << std::endl;
		std::cout << "\t" << properties_.axis_x << " " << properties_.axis_y << " " << properties_.axis_z << "("
			<< std::sqrt(std::pow(properties_.axis_x, 2) + std::pow(properties_.axis_y, 2) + std::pow(properties_.axis_z, 2)) << ")" << std::endl;
	}


	///

	/* Receives a PointCloud cut by reference and fits a sub-primitive to it. */
	void Primitive3Cuboid::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		std::vector<Primitive3d1*> subprim_arr;
		size_t n_subprims(0), n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_subprims < MAX_SUBPRIMS) {
			size_t cur_size(cloud->points.size());

			Primitive3Line* cut_prim(new Primitive3Line);
			cut_prim->fit(cloud, false);
			//*cloud_ += *cut_prim->getPointCloud();
			subprim_arr.push_back(cut_prim);
			++n_subprims;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cur_size << std::endl;
		}
		subprims_.push_back(subprim_arr);
	}

	/* Receives a PointCloud cut and a segmentation object by reference and extracts/segments it by fitting to it. */
	void Primitive3Cuboid::cut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SACSegmentation<pcl::PointXYZ>* seg)
	{
		std::vector<Primitive3d1*> subprim_arr;
		size_t n_subprims(0), n_pts(cloud->points.size());
		while (cloud->points.size() > 0.3 * n_pts && n_subprims < MAX_SUBPRIMS) {
			size_t cur_size(cloud->points.size());

			Primitive3Line* cut_prim(new Primitive3Line);
			cut_prim->fit(cloud, seg);
			//*cloud_ += *cut_prim->getPointCloud();
			subprim_arr.push_back(cut_prim);
			++n_subprims;
			std::cout << "\t" << cut_prim->getPointCloud()->size() << "/" << cur_size << std::endl;
		}
		subprims_.push_back(subprim_arr);
	}

	void Primitive3Cuboid::heuristic_laser_array_single()
	{
		/* WARNING!
		 * This implementation does not guarantee a full estimation of the geometric primitive.
		 */
		float cube_width(0.000), cube_height(0.001), cube_depth(0.001);
		Eigen::Vector3f e0_axis({ 0.0, 0.0, 0.0 }), e1_axis({ 0.0, 0.0, 0.0 }), z_axis({ 0.0, 0.0, 0.0 });
		Eigen::Matrix3f mori;
		Eigen::Vector3f face_center({ 0.0, 0.0, 0.0 }), cube_center({ 0.0, 0.0, 0.0 });

		// Computing the boundaries of the cut primitives
		//
		// [   ][-----][   ][   ]
		//

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<std::array<float, 2>> bounds_horizontal;
		for (int i(0); i < subprims_[0].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[0][i]->getPointCloud()));
			bounds_horizontal.push_back({ bounds_temp[0] ,bounds_temp[1] });
		}

		// Finding the "front face" spanning the center '-'
		int horizontal_idx(-1);
		for (int k1(0); k1 < bounds_horizontal.size(); ++k1) {
			if (bounds_horizontal[k1][0] <= 0.0 && 0.0 < bounds_horizontal[k1][1]) {
				horizontal_idx = k1;
				break;
			}
		}

		if (horizontal_idx != -1) {
			std::cout << " hori_idx: " << horizontal_idx << std::endl;

			// Find the centroid of the points in the line primitive
			Eigen::Vector3f point(
				subprims_[0][horizontal_idx]->getCoefficients()->values[0],
				subprims_[0][horizontal_idx]->getCoefficients()->values[1],
				subprims_[0][horizontal_idx]->getCoefficients()->values[2]
			);
			Eigen::Vector3f h_dir(
				subprims_[0][horizontal_idx]->getCoefficients()->values[3],
				subprims_[0][horizontal_idx]->getCoefficients()->values[4],
				subprims_[0][horizontal_idx]->getCoefficients()->values[5]
			);

			*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
			std::cout << "LINE\n" << std::endl;
			*cloud_ = *cloud_suprims;

			Eigen::Vector3f v_dir(0.0, cube_height, 0.0);

			// Find the cube_face center
			Eigen::Vector3f face_center(point.x() + h_dir.x() * 0.5, point.y() + h_dir.y() * 0.5, point.z() + h_dir.z() * 0.5);

			// Find the cube_face normal
			Eigen::Vector3f face_normal(h_dir.cross(v_dir));
			face_normal.normalize();
			if (face_normal.z() > 0.0) {
				face_normal[0] = -face_normal.x();
				face_normal[1] = -face_normal.y();
				face_normal[2] = -face_normal.z();
			}


			// Find the cuboid primitive parameters
			// Cuboid z
			cube_height = v_dir.norm();
			z_axis = v_dir;
			z_axis.normalize();
			z_axis *= cube_height / 2;
			// Cuboid e1
			cube_width = h_dir.norm();
			e1_axis = h_dir;
			e1_axis.normalize();
			e1_axis *= cube_width / 2;
			// Cuboid e0
			e0_axis = face_normal;
			e0_axis.normalize();
			e0_axis *= cube_depth / 2;

			cube_center = face_center + face_normal * cube_depth / 2;

			// Since width (plane e0) <= height (plane e1)
			// (assumption made for the cuboid)
			properties_.e0_x = e0_axis(0);
			properties_.e0_y = e0_axis(1);
			properties_.e0_z = e0_axis(2);
			properties_.e1_x = e1_axis(0);
			properties_.e1_y = e1_axis(1);
			properties_.e1_z = e1_axis(2);
			properties_.axis_x = z_axis(0);
			properties_.axis_y = z_axis(1);
			properties_.axis_z = z_axis(2);

			// Transformation matrix
			e0_axis.normalize();
			mori(0, 0) = e0_axis(0);
			mori(1, 0) = e0_axis(1);
			mori(2, 0) = e0_axis(2);
			e1_axis.normalize();
			mori(0, 1) = e1_axis(0);
			mori(1, 1) = e1_axis(1);
			mori(2, 1) = e1_axis(2);
			z_axis.normalize();
			mori(0, 2) = z_axis(0);
			mori(1, 2) = z_axis(1);
			mori(2, 2) = z_axis(2);

			Eigen::Quaternionf quat(mori);

			//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
			coefficients_->values[0] = cube_center.x(); //Tx
			coefficients_->values[1] = cube_center.y(); //Ty
			coefficients_->values[2] = cube_center.z(); //Tz
			coefficients_->values[3] = quat.x(); //Qx
			coefficients_->values[4] = quat.y(); //Qy
			coefficients_->values[5] = quat.z(); //Qz
			coefficients_->values[6] = quat.w(); //Qw
			coefficients_->values[7] = cube_depth; //cube_width;
			coefficients_->values[8] = cube_width; //cube_height;
			coefficients_->values[9] = cube_height; //cube_depth;
		}
		return;
	}

	void Primitive3Cuboid::heuristic_laser_array_cross()
	{
		float cube_width(0.000), cube_height(0.000), cube_depth(0.001);
		Eigen::Vector3f e0_axis({ 0.0, 0.0, 0.0 }), e1_axis({ 0.0, 0.0, 0.0 }), z_axis({ 0.0, 0.0, 0.0 });
		Eigen::Matrix3f mori;
		Eigen::Vector3f face_center({ 0.0, 0.0, 0.0 }), cube_center({ 0.0, 0.0, 0.0 });

		// Computing the boundaries of the line primitives
		//   [-]
		//   [-]
		// [-----][--------]
		//   [-]
		//   [-]
		//   [-]

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<std::array<float, 2>> bounds_horizontal; // 0 deg
		for (int i(0); i < subprims_[0].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[0][i]->getPointCloud()));
			bounds_horizontal.push_back({ bounds_temp[0] ,bounds_temp[1] });
		}
		std::vector<std::array<float, 2>> bounds_vertical; // 90 deg
		for (int i(0); i < subprims_[1].size(); ++i) {
			std::array<float, 6> bounds_temp(getPointCloudRanges(*subprims_[1][i]->getPointCloud()));
			bounds_vertical.push_back({ bounds_temp[2] ,bounds_temp[3] });
		}

		// Finding the front cube_face spaned by the intersection '+'
		int vertical_idx(-1), horizontal_idx(-1);
		for (int k1(0); k1 < bounds_vertical.size(); ++k1) {
			for (int k2(0); k2 < bounds_horizontal.size(); ++k2) {
				if (bounds_vertical[k1][0] <= 0.0 && 0.0 < bounds_vertical[k1][1] && bounds_horizontal[k2][0] <= 0.0 && 0.0 < bounds_horizontal[k2][1]) {
					vertical_idx = k1;
					horizontal_idx = k2;
					break;
				}
			}
		}

		if (vertical_idx != -1 && horizontal_idx != -1) {
			std::cout << "vert_idx: " << vertical_idx << " hori_idx: " << horizontal_idx << std::endl;

			// Find the centroid of the points in the line primitive
			Eigen::Vector3f h_point(
				subprims_[0][horizontal_idx]->getCoefficients()->values[0],
				subprims_[0][horizontal_idx]->getCoefficients()->values[1],
				subprims_[0][horizontal_idx]->getCoefficients()->values[2]
			);
			Eigen::Vector3f h_dir(
				subprims_[0][horizontal_idx]->getCoefficients()->values[3],
				subprims_[0][horizontal_idx]->getCoefficients()->values[4],
				subprims_[0][horizontal_idx]->getCoefficients()->values[5]
			);
			Eigen::Vector3f h_center(h_point.x() + h_dir.x() * 0.5, h_point.y() + h_dir.y() * 0.5, h_point.z() + h_dir.z() * 0.5);

			*cloud_suprims += *subprims_[0][horizontal_idx]->getPointCloud();
			std::cout << "LINE\n" << std::endl;			

			Eigen::Vector3f v_point(
				subprims_[1][vertical_idx]->getCoefficients()->values[0],
				subprims_[1][vertical_idx]->getCoefficients()->values[1],
				subprims_[1][vertical_idx]->getCoefficients()->values[2]
			);
			Eigen::Vector3f v_dir(
				subprims_[1][vertical_idx]->getCoefficients()->values[3],
				subprims_[1][vertical_idx]->getCoefficients()->values[4],
				subprims_[1][vertical_idx]->getCoefficients()->values[5]
			);
			Eigen::Vector3f v_center(v_point.x() + v_dir.x() * 0.5, v_point.y() + v_dir.y() * 0.5, v_point.z() + v_dir.z() * 0.5);

			*cloud_suprims += *subprims_[1][vertical_idx]->getPointCloud();
			std::cout << "LINE\n" << std::endl;

			*cloud_ = *cloud_suprims;

			// Find the cube_face center
			Eigen::Vector3f vec(v_center.x() - h_center.x(), v_center.y() - h_center.y(), v_center.z() - h_center.z());
			Eigen::Vector3f face_center(
				h_center.x() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.x(),
				h_center.y() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.y(),
				h_center.z() + v_dir.dot(vec) / std::pow(v_dir.norm(), 2) * v_dir.z()
			);
			
			// Find the cube_face normal
			Eigen::Vector3f face_normal(h_dir.cross(v_dir));
			face_normal.normalize();
			if (face_normal.z() > 0.0) {
				face_normal[0] = -face_normal.x();
				face_normal[1] = -face_normal.y();
				face_normal[2] = -face_normal.z();
			}

			// Find the cuboid primitive parameters
			//// Select the larger dimension/direction as the Z-axis
			if (h_dir.norm() >= v_dir.norm()) {
				// Cuboid z
				cube_height = h_dir.norm();
				z_axis = h_dir;
				z_axis.normalize();
				z_axis *= cube_height / 2;
				// Cuboid e1
				cube_width = v_dir.norm();
				e1_axis = v_dir;
				e1_axis.normalize();
				e1_axis *= cube_width / 2;
			}
			else {
				// Cuboid z
				cube_height = v_dir.norm();
				z_axis = v_dir;
				z_axis.normalize();
				z_axis *= cube_height / 2;
				// Cuboid e1
				cube_width = h_dir.norm();
				e1_axis = h_dir;
				e1_axis.normalize();
				e1_axis *= cube_width / 2;
			}
			// Cuboid e0
			e0_axis = face_normal;
			e0_axis.normalize();
			e0_axis *= cube_depth / 2;

			cube_center = face_center + face_normal * cube_depth / 2;

			// Since width (plane e0) <= height (plane e1)
			// (assumption made for the cuboid)
			properties_.e0_x = e0_axis(0);
			properties_.e0_y = e0_axis(1);
			properties_.e0_z = e0_axis(2);
			properties_.e1_x = e1_axis(0);
			properties_.e1_y = e1_axis(1);
			properties_.e1_z = e1_axis(2);
			properties_.axis_x = z_axis(0);
			properties_.axis_y = z_axis(1);
			properties_.axis_z = z_axis(2);

			// Transformation matrix
			e0_axis.normalize();
			mori(0, 0) = e0_axis(0);
			mori(1, 0) = e0_axis(1);
			mori(2, 0) = e0_axis(2);
			e1_axis.normalize();
			mori(0, 1) = e1_axis(0);
			mori(1, 1) = e1_axis(1);
			mori(2, 1) = e1_axis(2);
			z_axis.normalize();
			mori(0, 2) = z_axis(0);
			mori(1, 2) = z_axis(1);
			mori(2, 2) = z_axis(2);

			Eigen::Quaternionf quat(mori);

			//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
			coefficients_->values[0] = cube_center.x(); //Tx
			coefficients_->values[1] = cube_center.y(); //Ty
			coefficients_->values[2] = cube_center.z(); //Tz
			coefficients_->values[3] = quat.x(); //Qx
			coefficients_->values[4] = quat.y(); //Qy
			coefficients_->values[5] = quat.z(); //Qz
			coefficients_->values[6] = quat.w(); //Qw
			coefficients_->values[7] = cube_depth; //cube_width;
			coefficients_->values[8] = cube_width; //cube_height;
			coefficients_->values[9] = cube_height; //cube_depth;
		}
		return;
	}

	void Primitive3Cuboid::heuristic_laser_array_star()
	{
		float cube_width(0.000), cube_height(0.000), cube_depth(0.001);
		Eigen::Vector3f e0_axis({ 0.0, 0.0, 0.0 }), e1_axis({ 0.0, 0.0, 0.0 }), z_axis({ 0.0, 0.0, 0.0 });
		Eigen::Matrix3f mori;
		Eigen::Vector3f face_center({ 0.0, 0.0, 0.0 }), cube_center({ 0.0, 0.0, 0.0 });

		// Computing the boundaries of the line primitives
		//   \ [-] /
		//    \[-]/
		//	 [-----][--------]
		//    /[-]\
		//   / [-] \
		//  /  [-]  \
		
		std::array<pcl::PointXYZ, 4> normals = {
			pcl::PointXYZ(1, 0, 0), // 0 deg
			pcl::PointXYZ(1, 1, 0), // 45 deg
			pcl::PointXYZ(0, 1, 0), // 90 deg
			pcl::PointXYZ(-1, 1, 0) // 135 deg
		};
		// Key-points at the extremes of each subprim crossin the origin will be used by the heuristic
		std::array<pcl::PointXYZ, 8> keypoints;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_face(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suprims(new pcl::PointCloud<pcl::PointXYZ>);

		std::array<int, 4> idx({ -1, -1, -1, -1 }); // idx of the selected sub-primitive within each cut (if found)
		std::array<std::array<float, 2>, 4> bounds;
		for (int k(0); k < 4; ++k) {
			// Iterate over the laser cuts
			for (int i(0); i < subprims_[k].size(); ++i) {
				// Iterate over the fitted subprim clouds
				pcl::PointXYZ min, max;
				std::array<float, 2> bounds_temp(getPointCloudExtremes(*subprims_[k][i]->getPointCloud(), pcl::PointXYZ(0, 0, 0), normals[k], min, max));
				bounds[i] = { bounds_temp[0], bounds_temp[1] };

				// Find the front face spaned by the intersection '+'
				if (bounds_temp[0] < 0.0 && bounds_temp[1] > 0.0) {
					keypoints[k + 4] = min; // ex. negative x-axis -> 4
					keypoints[k] = max; // ex. positive x-axis -> 0					

					if (subprims_[k][i]->getCoefficients()->values.size() == 6) {
						// Subprimitive is a Line
						idx[k] = i;
						*cloud_face += *subprims_[k][i]->getPointCloud();
						*cloud_suprims += *subprims_[k][i]->getPointCloud();
						std::cout << "LINE\n" << std::endl;
					}
				}
			}
		}
		*cloud_ = *cloud_suprims;

		if (subprims_.size() == 4 && idx[0] != -1 && idx[1] != -1 && idx[2] != -1 && idx[3] != -1) {
			std::cout << "idx_0: " << idx[0] << " idx_1: " << idx[1] << " idx_2: " << idx[2] << " idx_3: " << idx[3] << std::endl;

			// Find the plane that contains the extreme keypoints of the lines
			std::array<Eigen::Vector3f, 4> plane_pts; // (four keypoints at most)
			int p_counter(0);
			for (int k(0); k < 4; ++k) {
					// Alternate the selection of min or max points
					if (k % 2 == 0) {
						plane_pts[p_counter] = Eigen::Vector3f(keypoints[k].x, keypoints[k].y, keypoints[k].z);
						++p_counter;
					}
					else {
						plane_pts[p_counter] = Eigen::Vector3f(keypoints[k + 4].x, keypoints[k + 4].y, keypoints[k + 4].z);
						++p_counter;
					}
			}
			
			// Normal always points towards the camera -Z (assumes objects are convex)
			Eigen::Vector3f face_normal = (plane_pts[1] - plane_pts[0]).cross((plane_pts[2] - plane_pts[1]));
			face_normal.normalize();
			if (face_normal(2) > 0) {
				face_normal(0) = -face_normal(0);
				face_normal(1) = -face_normal(1);
				face_normal(2) = -face_normal(2);
			}

			// Project all lines keypoints onto the plane
			Eigen::Vector3f center(keypoints[0].x, keypoints[0].y, keypoints[0].z);
			for (size_t i(0); i < keypoints.size(); ++i) {
				Eigen::Vector3f kp(keypoints[i].x, keypoints[i].y, keypoints[i].z), kp_proj(0.0, 0.0, 0.0);
				kp_proj = kp - (kp - center).dot(face_normal) * face_normal / face_normal.squaredNorm();
				keypoints[i] = pcl::PointXYZ(kp_proj(0), kp_proj(1), kp_proj(2));
			}

			// Longitudinal "direction" vector to be found by the heuristic
			// (this orientation heuristic is based on the projected keypoints)

			// Find the centroid of the points in the line primitive
			Eigen::Vector3f va_dir({0.0, 0.0, 0.0}); // first vector to be found by the heuristic
			for (int i(0); i < keypoints.size(); ++i) {
				Eigen::Vector3f kp, kp_prev, kp_next;
				kp(0) = keypoints[i].x;
				kp(1) = keypoints[i].y;
				kp(2) = keypoints[i].z;
				if (i == 0) {
					kp_prev(0) = keypoints[keypoints.size()-1].x;
					kp_prev(1) = keypoints[keypoints.size()-1].y;
					kp_prev(2) = keypoints[keypoints.size()-1].z;
				}
				else {
					kp_prev(0) = keypoints[i-1].x;
					kp_prev(1) = keypoints[i-1].y;
					kp_prev(2) = keypoints[i-1].z;
				}
				if (i == keypoints.size()-1) {
					kp_next(0) = keypoints[0].x;
					kp_next(1) = keypoints[0].y;
					kp_next(2) = keypoints[0].z;
				}
				else {
					kp_next(0) = keypoints[i + 1].x;
					kp_next(1) = keypoints[i + 1].y;
					kp_next(2) = keypoints[i + 1].z;
				}

				// Check for parallel vectors
				Eigen::Vector3f v_prev = kp - kp_prev;
				Eigen::Vector3f v_next = kp_next - kp;
				float angle = std::acos(v_prev.dot(v_next) / (v_prev.norm() * v_next.norm()));				
				
				// Estimate the direction_vector
				float ang_threshold(10.0);
				if (angle < ang_threshold * M_PI / 180.0) {
					va_dir = kp_next - kp_prev;
					va_dir.normalize();
					std::cout << "Angle between vectors: " << angle * 180.0 / M_PI << std::endl;
					break;
				}
			}

			// The direction_vector has been found:
			if (va_dir.norm() > 0.0) {
				// 'va_dir' can be either the 1st or 2nd principal component of the cuboid face

				// Remaining direction vector of the heuristic
				Eigen::Vector3f vb_dir = face_normal.cross(va_dir);
				vb_dir.normalize();

				// Find the cube_face center     >! Note: may not be accurate!<
				for (auto p : cloud_face->points) {
					face_center(0) += p.x;
					face_center(1) += p.y;
					face_center(2) += p.z;
				}
				face_center /= cloud_face->size();

				// Find the cuboid primitive parameters
				std::array<float, 2> bounds_va(getPointCloudExtremes(*cloud_face, pcl::PointXYZ(face_center.x(), face_center.y(), face_center.z()), pcl::PointXYZ(va_dir.x(), va_dir.y(), va_dir.z())));
				float length_va = bounds_va[1] - bounds_va[0];
				std::array<float, 2> bounds_vb(getPointCloudExtremes(*cloud_face, pcl::PointXYZ(face_center.x(), face_center.y(), face_center.z()), pcl::PointXYZ(vb_dir.x(), vb_dir.y(), vb_dir.z())));
				float length_vb = bounds_vb[1] - bounds_vb[0];


				// Find the cuboid primitive parameters
				//// Select the larger dimension/direction as the Z-axis
				/*
				 *    ___________
				 *   |   Z ^    ||
				 *   |     |    ||
				 *   |     |    ||
				 *   |     ---> ||
				 *   |    /     ||
				 *   |  e0	    ||
				 *   |__________/
				 */

				if (length_va >= length_vb) {
					// Cuboid z
					cube_height = length_va;
					z_axis = va_dir;
					z_axis.normalize();
					z_axis *= cube_height / 2;
					// Cuboid e1
					cube_width = length_vb;
					e1_axis = -vb_dir; // sign change
					e1_axis.normalize();
					e1_axis *= cube_width / 2;
				}
				else {
					// Cuboid z
					cube_height = length_vb;
					z_axis = vb_dir;
					z_axis.normalize();
					z_axis *= cube_height / 2;
					// Cuboid e1
					cube_width = length_va;
					e1_axis = va_dir;
					e1_axis.normalize();
					e1_axis *= cube_width / 2;
				}
				// Cuboid e0
				//cube_depth = length_normal;
				e0_axis = face_normal;
				e0_axis.normalize();
				e0_axis *= cube_depth / 2;

				cube_center = face_center + face_normal * cube_depth / 2;
			}

			// Since width (plane e0) <= height (plane e1)
			// (assumption made for the cube)
			properties_.e0_x = e0_axis(0);
			properties_.e0_y = e0_axis(1);
			properties_.e0_z = e0_axis(2);
			properties_.e1_x = e1_axis(0);
			properties_.e1_y = e1_axis(1);
			properties_.e1_z = e1_axis(2);
			properties_.axis_x = z_axis(0);
			properties_.axis_y = z_axis(1);
			properties_.axis_z = z_axis(2);

			// Transformation matrix
			e0_axis.normalize();
			mori(0, 0) = e0_axis(0);
			mori(1, 0) = e0_axis(1);
			mori(2, 0) = e0_axis(2);
			e1_axis.normalize();
			mori(0, 1) = e1_axis(0);
			mori(1, 1) = e1_axis(1);
			mori(2, 1) = e1_axis(2);
			z_axis.normalize();
			mori(0, 2) = z_axis(0);
			mori(1, 2) = z_axis(1);
			mori(2, 2) = z_axis(2);

			Eigen::Quaternionf quat(mori);

			//Cube coefficients(Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
			coefficients_->values[0] = cube_center.x(); //Tx
			coefficients_->values[1] = cube_center.y(); //Ty
			coefficients_->values[2] = cube_center.z(); //Tz
			coefficients_->values[3] = quat.x(); //Qx
			coefficients_->values[4] = quat.y(); //Qy
			coefficients_->values[5] = quat.z(); //Qz
			coefficients_->values[6] = quat.w(); //Qw
			coefficients_->values[7] = cube_depth; //cube_width;
			coefficients_->values[8] = cube_width; //cube_height;
			coefficients_->values[9] = cube_height; //cube_depth;
		}
	}

	/* Checks if the heuristic is valid. */
	bool Primitive3Cuboid::is_heuristic_valid()
	{
		if (coefficients_->values[7] == 0.0 || coefficients_->values[8] == 0.0 || coefficients_->values[9] == 0.0) {
			this->reset();
			return false;
		}
		
		/* Checks if the cut sub-primitives are valid. */
		if (!subprims_.empty()) {
			for (auto arr : subprims_) {
				for (auto sp : arr) {
					if (sp->getPointCloud()->empty()) {
						this->reset();
						return false;
					}
				}
			}
		}

		isempty_ = false;
		return true;
	}
}