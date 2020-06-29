#pragma once
#include "sensor.h"
#include "primitive.h"

#include <vector>

class Solver
{
public:
	Solver() {}
	//Solver(robin::Primitive*);
	~Solver(){}

	virtual void addSensor(robin::Sensor*) {}

	virtual void setPrimitive(robin::Primitive*) {}

protected:
	std::vector<robin::Sensor*> sensors_;
	robin::Primitive* primitive_ = nullptr;
};