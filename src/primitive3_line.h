#pragma once
#include "primitive3.h"

namespace robin
{
	class Primitive3Line :
		public Primitive3d1
	{
	public:
		struct Properties getProperties();
	
	protected:
		struct Properties {
			float length;
		} properties_;
	};
}