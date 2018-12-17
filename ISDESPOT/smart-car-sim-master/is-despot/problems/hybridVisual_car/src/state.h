#ifndef PED_STATE_H
#define PED_STATE_H
#include "coord.h"
#include "param.h"
#include "core/pomdp.h"
#include "disabled_util.h"
#include <vector>
#include <utility>
using namespace std;

struct PedStruct {
	COORD pos;
	int goal;
	int id;
	double vel;

	PedStruct(){
        vel = ModelParams::PED_SPEED;
    }
	PedStruct(COORD pos_, int goal_, int id_) {
		pos = pos_;
		goal = goal_;
		id = id_;
        vel = ModelParams::PED_SPEED;
	}
};

class Pedestrian
{
public:
	double x, y;
	int id = -1;
	double last_update;

	Pedestrian() {}
	Pedestrian(double x_,double y_,int id_) {
		x = x_;
		y = y_;
		id = id_;
    }
	Pedestrian(double x_, double y_) {
		x = x_;
		y = y_;
	}
};

struct CarStruct {
	int pos;
	double vel;
	double dist_travelled;
};

class PomdpState : public State {
public:
	CarStruct car;
	int num;
	PedStruct peds[ModelParams::N_PED_IN];
	PomdpState() {}

	string text() const {
		return concat(car.vel);
	}
};

class PomdpStateWorld : public State {
public:
	CarStruct car;
	int num;
	PedStruct peds[ModelParams::N_PED_WORLD];
	PomdpStateWorld() {}

	string text() const {
		return concat(car.vel);
	}
};

#endif
