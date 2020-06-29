#pragma once
/*#include "base.hpp"
#include "sensor.hpp"
#include "primitive.hpp"

#include <vector>

#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace robin {

	class Hand :
		public Base
	{
	public:

		Hand();
		~Hand();
		
		void addSensor(robin::Sensor*);

		void addPrimitive(robin::Primitive&);

		robin::Primitive getSegmented();

		void activate();

	protected:
		void startSensors(bool);

		void segment();

		void rotate();

	private:
		std::array<float, 6> _configstate;

		std::vector<robin::Sensor*> _sensors;
		std::vector<robin::Primitive> _primitives;

		pcl::visualization::PCLVisualizer::Ptr viewer_;
		pcl::visualization::PCLVisualizer::Ptr Hand::initVisualizer();

		bool iskeypressed(unsigned);

	};

}*/

