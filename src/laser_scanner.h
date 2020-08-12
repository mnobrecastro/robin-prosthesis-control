#pragma once
#include "sensor3.h"
#include "camera_depth.h"

#include <array>

#include <Eigen/StdVector>
#include <pcl/common/io.h>

namespace robin {

	class LaserScanner :
		public Sensor3
	{
	public:
		LaserScanner() = delete;

		/* Create a LaserScanner from a depth camera by applying a planar cut to the PointCloud.
		 * Params of the cut must follow the canonical form of a planar equation (a*x + b*y + c*z + d = 0) */
		LaserScanner(CameraDepth* cam, float a, float b, float c, float d, float tol);

		/* Create a LaserScanner from a depth camera by applying a planar cut to the PointCloud.
		 * Params of the cut must follow the canonical form of a linear equation (a*x + b = 0) about the xOy plane. */
		//LaserScanner(CameraDepth* cam, float a, float b, float tol);
		
		~LaserScanner() {}

	protected:
		CameraDepth* camera_ = nullptr;

		struct PlanarCutParameters {
			/* Canonical equation of a planar cut:
			 *		a*x + b*y + c*z + d = 0
			 * where:
			 *		normal n = (a,b,c)
			 *		d = -(a*x0 + b*y0 + c*z0)
			 */
			float a = 0.0;
			float b = 0.0;
			float c = 0.0;
			float d = 0.0;
			float tol = 0.0;
		} params_;


		void fromParent(const pcl::PointCloud<pcl::PointXYZ>& cloud);
		void slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_slice);
		float projectionDistance(const pcl::PointXYZ& p);

	private:
		friend class RealsenseD400;
	};

}