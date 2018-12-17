#ifndef HYBRIDDESPOT_H
#define HYBRIDDESPOT_H

#include "solver/despot.h"

class HybridDESPOT: public DESPOT {

public:
	HybridDESPOT(DSPOMDP* pomdp, ScenarioLowerBound*& lower, ScenarioUpperBound*& upper):
		DESPOT(pomdp, lower, upper)
	{

	}
};

#endif
