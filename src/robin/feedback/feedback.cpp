#include "feedback.h"

namespace robin
{
	namespace feedback
	{
		Feedback::Feedback()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_hull_ = cloud;
		}

		Feedback::~Feedback() {}
		
		void Feedback::addTactor(Tactor* s)
		{
			tactor_ = s;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::getPointCloud() const
		{
			return cloud_hull_;
		}

		void Feedback::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
		{
			for (int i(0); i < cloud_hull_->points.size(); ++i) {
				if (i == 0)
					viewer->addLine<pcl::PointXYZ>(cloud_hull_->points[cloud_hull_->points.size()-1], cloud_hull_->points[i], 1.0, 1.0, 1.0, "hull" + std::to_string(i));
				else
					viewer->addLine<pcl::PointXYZ>(cloud_hull_->points[i-1], cloud_hull_->points[i], 1.0, 1.0, 1.0, "hull" + std::to_string(i));
			}
			if (!in_hull_) {
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(0.0,0.0,0.0), pHull_, 1.0, 0.0, 1.0, "pHull");
				std::cout << pHull_ << std::endl;
			}

			return;
		}

		void Feedback::addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, robin::FEEDBACK_CLOUD fc)
		{
			// Reset the feedback's PointCloud and pHull
			cloud_hull_->clear();
			pHull_ = pcl::PointXYZ(-1.0, 0.0, 0.0);
			in_hull_ = false;
			
			// Duplicate the pointcloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

			switch (fc)
			{
			case robin::FEEDBACK_CLOUD::DIST_TO_HULL:
				// Transform the 'cloud' as a projection into the xOy plane
				for (auto& p : cloud_cpy->points) { p.z = 0.0; }

				// Obtain the convexhull of the projected cloud
				cloud_hull_ = this->quick_hull_2d(cloud_cpy);

				if (cloud_hull_->points.size() > 2) {
					// Check whether the origin points lies inside the hull
					in_hull_ = this->is_inside_hull(cloud_hull_, pcl::PointXYZ(0, 0, 0), pHull_);
				}
				break;

			case robin::FEEDBACK_CLOUD::DIST_TO_COM:
				// Transform the 'cloud' as a projection into the xOy plane
				for (auto& p : cloud_cpy->points) { p.z = 0.0; }

				// Calculate the Center-of-Mass of the projected cloud
				float xbar(0.0), ybar(0.0), zbar(0.0);
				for (auto p : cloud->points) {
					xbar += p.x;
					ybar += p.y;
					zbar += p.z;
				}
				xbar /= cloud->points.size();
				ybar /= cloud->points.size();
				zbar /= cloud->points.size();
				break;

			}

			return;
		}

		void Feedback::addPrimitive3(const robin::Primitive3d3* prim, robin::FEEDBACK_PRIM fp)
		{
			robin::PRIM_TYPE prim_type;
			switch (fp)
			{
			case robin::FEEDBACK_PRIM::TYPE:
				
				if (typeid(*prim) == typeid(robin::Primitive3Sphere)) {
					prim_type = robin::PRIM_TYPE::SPHERE;
				}
				else if (typeid(*prim) == typeid(robin::Primitive3Cylinder)) {
					prim_type = robin::PRIM_TYPE::CYLINDER;
				}
				else if (typeid(*prim) == typeid(robin::Primitive3Cuboid)) {
					prim_type = robin::PRIM_TYPE::CUBOID;
				}
				break;
			}

			prim_type_ = prim_type;
			return;
		}

		void Feedback::run()
		{
			if (!is_active_) {
				// Not active
				tactor_->setSample({ -1.0, 0.0 });
			}
			else {
				if (in_hull_) {
					// Use the primitive type to set a specific behaviour
					switch (prim_type_)
					{
					case robin::PRIM_TYPE::SPHERE:
						tactor_->setSample({ 0.0, 0.0 });//, 1);
						break;

					case robin::PRIM_TYPE::CYLINDER:
						tactor_->setSample({ 0.0, 0.0 });//, 2);
						break;

					case robin::PRIM_TYPE::CUBOID:
						tactor_->setSample({ 0.0, 0.0 });// , 3);
						break;

					default:
						tactor_->setSample({ -1.0, 0.0 });
					}
				}
				else {
					if (cloud_hull_->points.size() > 2) {
						// Rho
						float rho = std::sqrt(pHull_.x * pHull_.x + pHull_.y * pHull_.y);
						const float MAX_RHO = 0.050; //m					
						if (rho / MAX_RHO <= 1.0)
							rho = rho / MAX_RHO;
						else
							rho = 1.0;
						// Theta
						float theta = std::atan2(pHull_.y, pHull_.x);

						tactor_->setSample({ rho, theta });
					}
					else {
						tactor_->setSample({ -1.0, 0.0 });
					}
				}
			}
			return;
		}


		//

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::quick_hull_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
		{
			/* O(N.logN) */
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
			float tol(0.001); // to reduce the number of points in the hull

			if (cloud->empty()) {
				// Return if the set is empty				
				return hull;
			}
			if (cloud->points.size() == 1) {
				// Return the singular set
				hull->push_back(cloud->points[0]);
				return hull;
			}

			size_t L_idx(-1), R_idx(-1);
			pcl::PointXYZ pL(1e6, 0.0, 0.0), pR(-1e6, 0.0, 0.0);
			
			// Find the two extreme points (-x, +x)
			for (int i(0); i < cloud->points.size(); ++i) {
				// Leftmost highest
				if (cloud->points[i].x < pL.x - tol/2) {
					L_idx = i;
					pL = cloud->points[i];
				}
				else if (cloud->points[i].x >= pL.x - tol/2 && cloud->points[i].x <= pL.x + tol/2){
					// Select the highest within tolerance
					if (cloud->points[i].y > pL.y) {
						L_idx = i;
						pL = cloud->points[i];
					}
				}
				// Righgmost lowest
				if (cloud->points[i].x > pR.x + tol/2) {
					R_idx = i;
					pR = cloud->points[i];					
				}
				else if (cloud->points[i].x >= pR.x - tol/2 && cloud->points[i].x <= pR.x + tol/2) {
					// Select the lowest within tolerance
					if (cloud->points[i].y < pR.y) {
						R_idx = i;
						pR = cloud->points[i];
					}
				}
			}

			// Find right-side and left-side subclouds
			pcl::PointIndices::Ptr rightside(new pcl::PointIndices());
			pcl::PointIndices::Ptr leftside(new pcl::PointIndices());			
			for (int i(0); i < cloud->points.size(); ++i) {
				if (i != L_idx && i != R_idx) {
					pcl::PointXYZ vL(pL.x - cloud->points[i].x, pL.y - cloud->points[i].y, 0.0);
					pcl::PointXYZ vR(pR.x - cloud->points[i].x, pR.y - cloud->points[i].y, 0.0);
					// (pR->pL)
					float cross = vL.x * vR.y - vL.y * vR.x;
					if(cross > 0.0)	{
						rightside->indices.push_back(i);
					}
					else if (cross < 0.0) {
						leftside->indices.push_back(i);
					}
					else {
						// The point lies on the line
						// (do nothing)
					}
				}
			}
			
			// Extract the indexed points from the cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			extract.setInputCloud(cloud);
			extract.setIndices(leftside);
			extract.setNegative(false);
			extract.filter(*left_cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			extract.setInputCloud(cloud);
			extract.setIndices(rightside);
			extract.setNegative(false);
			extract.filter(*right_cloud);

			// Return the hull pointcloud ordered from the right-most vertex
			hull->points.push_back(pR);
			*hull += *this->find_hull_2d(right_cloud, pR, pL);
			hull->points.push_back(pL);
			*hull += *this->find_hull_2d(left_cloud, pL, pR);
			
			return hull;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::find_hull_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pR, pcl::PointXYZ pL) const
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);

			if (cloud->points.size() == 0) {
				// Return if the set is empty				
				return hull;
			}

			// Find the farthest point 'c' in the set
			// (search for the largest perp projection distance to segment RL)
			pcl::PointXYZ pC(0.0, 0.0, 0.0);
			float dist(0.0); // avoids collinear points
			bool found(false);
			Eigen::Vector3f u(pL.x - pR.x, pL.y - pR.y, 0.0);
			for (auto p : cloud->points) {
				Eigen::Vector3f v(p.x - pR.x, p.y - pR.y, 0.0);
				Eigen::Vector3f proj = v.dot(u) * u / u.squaredNorm();
				// Perpendicular projection
				if ((v - proj).norm() > dist) {
					pC = p;
					dist = (v - proj).norm();
					found = true;
				}
			}
			if (!found) {
				// Return if the unique point/s are neglegible due to small dist				
				return hull;
			}

			// Find right-side and left-side points to the triangle RCL
			pcl::PointIndices::Ptr leftside(new pcl::PointIndices());
			pcl::PointIndices::Ptr rightside(new pcl::PointIndices());
			for (int i(0); i < cloud->points.size(); ++i) {
				// Distance vectors from point p_i to the points R, C, L
				pcl::PointXYZ vR(pR.x - cloud->points[i].x, pR.y - cloud->points[i].y, 0.0);
				pcl::PointXYZ vC(pC.x - cloud->points[i].x, pC.y - cloud->points[i].y, 0.0);
				pcl::PointXYZ vL(pL.x - cloud->points[i].x, pL.y - cloud->points[i].y, 0.0);
				
				// Right w.r.t. the segment RC
				if (vC.x * vR.y - vC.y * vR.x > 0.0) {
					// Right-side w.r.t. the triangle RCL
					rightside->indices.push_back(i);
				}
				// Right w.r.t. the segment CL
				else if (vL.x * vC.y - vL.y * vC.x > 0.0) {
					// Left-side w.r.t. the triangle RCL
					leftside->indices.push_back(i);
				}
				else {
					// The point lies inside the triangle RCL
					// (do nothing)
				}
			}

			// Extract the indexed points from the cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			extract.setInputCloud(cloud);
			extract.setIndices(rightside);
			extract.setNegative(false);
			extract.filter(*right_cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			extract.setInputCloud(cloud);
			extract.setIndices(leftside);
			extract.setNegative(false);
			extract.filter(*left_cloud);
						
			// Return the partial hull pointcloud
			*hull += *find_hull_2d(right_cloud, pR, pC);
			hull->points.push_back(pC);
			*hull += *find_hull_2d(left_cloud, pC, pL);
			return hull;			
		}

		bool Feedback::is_inside_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr hull, const pcl::PointXYZ p, pcl::PointXYZ& pHull) const
		{
			/* O(logN) */

			// Check whether the point 'p' lies, clockwise, between p0-p1 and p0-pN points of the hull
			Eigen::Vector3f pP(p.x, p.y, p.z);
			Eigen::Vector3f pN(hull->points[hull->points.size()-1].x, hull->points[hull->points.size()-1].y, hull->points[hull->points.size()-1].z); //pN-1
			Eigen::Vector3f p0(hull->points[0].x, hull->points[0].y, hull->points[0].z); //p0
			Eigen::Vector3f p1(hull->points[1].x, hull->points[1].y, hull->points[1].z); //p1

			float cross_a = (p1-p0).cross(pP-p0).z();
			float cross_b = (p1-p0).cross(pN-p0).z();
			float cross_c = (pN-p0).cross(pP-p0).z();
			float cross_d = (pN-p0).cross(p1-p0).z();
			if ( (cross_a == 0 || (this->signum(cross_a) == 1 && this->signum(cross_b) == 1)) && (cross_c == 0 || (this->signum(cross_c) == -1 && this->signum(cross_d) == -1))) {
				std::cout << "cross_a: " << cross_a << " cross_b: " << cross_b << " cross_c: " << cross_c << " cross_d: " << cross_d << std::endl;
				std::cout << "sign_a: " << this->signum(cross_a) << " sign_b: " << this->signum(cross_b) << " sign_c: " << this->signum(cross_c) << " sign_d: " << this->signum(cross_d) << std::endl;
				std::cout << "-----------" << std::endl;

				if (cross_a == 0) {
					// Point 'p' lies above the line p0-p1
					pcl::PointXYZ p_p0p1, p_p1p2;
					float d_p0p1 = this->distance_to_lineseg(hull->points[0], hull->points[1], p, p_p0p1);
					float d_p1p2 = this->distance_to_lineseg(hull->points[1], hull->points[2], p, p_p1p2);

					if (d_p0p1 <= d_p1p2) {						
						if (d_p0p1 == 0) {
							// Point in the cHull
							return true;
						}
						else {
							pHull = p_p0p1;
							return false;
						}
					}
					else {						
						if (d_p1p2 == 0) {
							// Point in the cHull
							// (this should never be reached)
							return true;
						}
						else {
							pHull = p_p1p2;
							return false;
						}
					}
				}
				else {
					// Perform Binary Search until finding the last point pK to which p is clockwise (p lies at the left-side of p0pK)
					Eigen::Vector3f pK, pK1; // 'pK1' stands for p(K-1)
					size_t idx, idx_min(1), idx_max(hull->points.size()-2);

					bool found(false);
					while (!found) {
						// Stopping criterion
						if ((idx_min + idx_max) / 2 == idx)
							// (handle the integer division, which rounds towards zero)
							idx = (idx_min + idx_max) / 2 + 1;
						else
							idx = (idx_min + idx_max) / 2;

						pK = Eigen::Vector3f(hull->points[idx].x, hull->points[idx].y, hull->points[idx].z); // p_k
						pK1 = Eigen::Vector3f(hull->points[idx+1].x, hull->points[idx + 1].y, hull->points[idx + 1].z); // p_k+1
						
						if ( (pK - p0).cross(pP - p0).z() > 0.0 && (pK1 - p0).cross(pP - p0).z() <= 0.0) {
							// Point 'p' lies polarly between pK and pK1
							found = true;
							break;
						}
						else {
							if ((pK - p0).cross(pP - p0).z() > 0.0 && (pK1 - p0).cross(pP - p0).z() > 0.0) {
								// Point 'p' is further counter-clockwise (left) to pK and pK1
								idx_min = idx;
							}
							else if ((pK - p0).cross(pP - p0).z() < 0.0 && (pK1 - p0).cross(pP - p0).z() < 0.0) {
								// Point 'p' is further clockwise (right) to pK and pK1
								idx_max = idx;
							}
							else {
								// Point 'p' lies behind p0pN p0p1
								std::cerr << "THE POINT WAS NOT FOUND" << std::endl;
								break;
							}
						}						
					}

					if (found) {
						// Calculate the areas of the following triangles
						float area_p0pKpK1 = std::abs((pK - p0).cross(pK1 - p0).z()); // area of triangle p0,pK,pK1
						float area_pPp0pK = std::abs((p0 - pP).cross(pK - pP).z()); // area of triangle pP,p0,pK
						float area_pPpKpK1 = std::abs((pK - pP).cross(pK1 - pP).z()); // area of triangle pP,pK,pK1
						float area_pPpK1p0 = std::abs((pK1 - pP).cross(p0 - pP).z()); // area of triangle pP,pK1,p0
					
						std::cout << "Area: " << area_p0pKpK1 << " Sum: " << area_pPp0pK + area_pPpKpK1 + area_pPpK1p0 << std::endl;

						float area_diff = area_p0pKpK1 - (area_pPp0pK + area_pPpKpK1 + area_pPpK1p0);
						if (area_diff > -1e-6 && area_diff < 1e-6) {
							// Point 'p' lies inside the triangle p0,pK,pK1
							std::cout << "++++ FOUND INSIDE!" << std::endl;
							return true;
						}
						else {
							// Point 'p' lies outside the triangle p0,pK,pK1
							std::cout << "++++ FOUND OUTSIDE!" << std::endl;
							float d_pKpK1 = this->distance_to_lineseg(hull->points[idx], hull->points[idx + 1], p, pHull);
							return false;
						}
					}
					else {
						std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
						return false;
					}
				}
			}
			else {
				// The point 'p' lies outside the convex hull
				std::cout << "Outside the HULL!" << std::endl;
				pcl::PointXYZ p_p0p1, p_p0pN;
				float d_p0p1 = this->distance_to_lineseg(hull->points[0], hull->points[1], p, p_p0p1);
				float d_p0pN = this->distance_to_lineseg(hull->points[0], hull->points[hull->points.size()-1], p, p_p0pN);
				if (d_p0p1 <= d_p0pN)
					pHull = p_p0p1;
				else
					pHull = p_p0pN;
				return false;
			}
		}

		int Feedback::signum(float val) const
		{
			return (val > 0) - (val < 0);
		}

		float Feedback::distance_to_lineseg(const pcl::PointXYZ a, const pcl::PointXYZ b, const pcl::PointXYZ p, pcl::PointXYZ& pHull) const
		{
			float d(-1.0);
			Eigen::Vector3f pA(a.x, a.y, a.z);
			Eigen::Vector3f pB(b.x, b.y, b.z);
			Eigen::Vector3f pP(p.x, p.y, p.z);

			Eigen::Vector3f u = (pB - pA).normalized();
			float scalar = (pP - pA).dot(u);
			if (scalar < 0.0) {
				pHull = a;
				d = (pP - pA).norm();
				std::cout << "---> A" << std::endl;
			}
			else if (scalar > (pB - pA).norm()) {
				pHull = b;
				d = (pP - pB).norm();
				std::cout << "---> B" << std::endl;
			}
			else {
				Eigen::Vector3f pLine = scalar * u;
				pHull = pcl::PointXYZ(pA.x() + pLine.x(), pA.y() + pLine.y(), pA.z() + pLine.z());
				d = ((pP - pA) - scalar * u).norm(); // norm of the rejection vector
				std::cout << "---> AB" << std::endl;
			}

			return d;
		}
			
	}
}