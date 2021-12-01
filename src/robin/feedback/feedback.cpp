#include "feedback.h"

namespace robin
{
	namespace feedback
	{
		Feedback::Feedback()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_ = cloud;
		}
		
		void Feedback::addSensor(Sensor* s)
		{
			sensors_.push_back(s);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::getPointCloud() const
		{
			return cloud_;
		}

		void Feedback::visualize(pcl::visualization::PCLVisualizer::Ptr viewer) const
		{
			for (int i(0); i < cloud_->points.size(); ++i) {
				if (i == 0)
					viewer->addLine<pcl::PointXYZ>(cloud_->points[cloud_->points.size()-1], cloud_->points[i], 1.0, 1.0, 1.0, "hull" + std::to_string(i));
				else
					viewer->addLine<pcl::PointXYZ>(cloud_->points[i-1], cloud_->points[i], 1.0, 1.0, 1.0, "hull" + std::to_string(i));
			}
		}

		void Feedback::fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, robin::FEEDBACK_CLOUD fc)
		{
			// Duplicate the pointcloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

			switch (fc)
			{
			case robin::FEEDBACK_CLOUD::CLOSEST_POINT:
				// Transform the 'cloud' as a projection into the xOy plane
				for (auto& p : cloud_cpy->points) { p.z = 0.0; }

				// Obtain the convexhull of the projected cloud
				cloud_ = quick_hull_2d(cloud_cpy);

				// Check whether the origin points lies inside the hull
				//bool inside = is_inside_hull();



				break;

			case robin::FEEDBACK_CLOUD::CENTER_OF_MASS:
				// Transform the 'cloud' as a projection into the xOy plane
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

		void Feedback::fromPrimitive(const robin::Primitive* prim, robin::FEEDBACK_PRIM fp)
		{

		}



		//

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::quick_hull_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
		{
			/* O(N.logN) */
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);

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
				if (cloud->points[i].x < pL.x) {
					L_idx = i;
					pL = cloud->points[i];
				}
				else if (cloud->points[i].x > pR.x) {
					R_idx = i;
					pR = cloud->points[i];
				}
			}

			// Find right-side and left-side subclouds
			pcl::PointIndices::Ptr rightside(new pcl::PointIndices());
			pcl::PointIndices::Ptr leftside(new pcl::PointIndices());
			float tol(0.001); //0.0
			for (int i(0); i < cloud->points.size(); ++i) {
				if (i != L_idx && i != R_idx) {
					pcl::PointXYZ vL(cloud->points[L_idx].x - cloud->points[i].x, cloud->points[L_idx].y - cloud->points[i].y, 0.0);
					pcl::PointXYZ vR(cloud->points[R_idx].x - cloud->points[i].x, cloud->points[R_idx].y - cloud->points[i].y, 0.0);

					float cross = vL.x * vR.y - vL.y * vR.x;
					if(cross > tol)	{
						rightside->indices.push_back(i);
					}
					else if (cross < tol) {
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
			*hull += *find_hull_2d(right_cloud, pR, pL);
			hull->points.push_back(pL);
			*hull += *find_hull_2d(left_cloud, pL, pR);
			
			return hull;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr Feedback::find_hull_2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pR, pcl::PointXYZ pL)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);

			if (cloud->points.size() == 0) {
				// Return if the set is empty				
				return hull;
			}

			// Find the farthest point 'c' in the set
			// (search for the largest perp projection distance to segment LR)
			pcl::PointXYZ pC(0.0, 0.0, 0.0);
			float dist(0.001);
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
			float tol(0.001); //0.0
			for (int i(0); i < cloud->points.size(); ++i) {
				// Distance vectors from point p_i to the points R, C, L
				pcl::PointXYZ vR(pR.x - cloud->points[i].x, pR.y - cloud->points[i].y, 0.0);
				pcl::PointXYZ vC(pC.x - cloud->points[i].x, pC.y - cloud->points[i].y, 0.0);
				pcl::PointXYZ vL(pL.x - cloud->points[i].x, pL.y - cloud->points[i].y, 0.0);
				
				// Right w.r.t. the segment RC
				if (vC.x * vR.y - vC.y * vR.x > tol) {
					// Right-side w.r.t. the triangle RCL
					rightside->indices.push_back(i);
				}
				// Right w.r.t. the segment CL
				else if (vL.x * vC.y - vL.y * vC.x > tol) {
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
			*hull += *find_hull_2d(right_cloud, pC, pR);
			hull->points.push_back(pC);
			*hull += *find_hull_2d(left_cloud, pL, pC);
			return hull;			
		}

		bool Feedback::is_inside_hull(pcl::PointXYZ p, pcl::PointXYZ& d)
		{
			/* O(logN) */


			return true;
		}
	}
}