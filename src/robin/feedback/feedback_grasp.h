#include "feedback.h"

namespace robin
{
	namespace feedback
	{
		class FeedbackGrasp :
			public Feedback
		{
		public:
			FeedbackGrasp() {}
			~FeedbackGrasp() {}

			void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {}

			void fromPrimitive(const robin::Primitive* prim) {}

		protected:
			
		};
	}
}