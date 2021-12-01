#include "feedback.h"

namespace robin
{
	namespace feedback
	{
		class FeedbackAim :
			public Feedback
		{
		public:
			FeedbackAim() {}
			~FeedbackAim() {}

			void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

			void fromPrimitive(const robin::Primitive* prim) {}

		protected:
			
		};
	}
}