
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include<string>
#include"SinCosLookupTable.cpp"

namespace ModelParams {
    extern int BELIEF_ANGLE_DISCRETIZATION;

	extern double CRASH_PENALTY;
	extern double REWARD_FACTOR_VEL;
	extern double VEL_MAX;
	extern double NOISE_GOAL_ANGLE;
	extern double REWARD_BASE_CRASH_VEL;
	extern double BELIEF_SMOOTHING;
	extern double NOISE_ROBVEL;
	extern double COLLISION_DISTANCE;
	extern double COLLISION_SIDE_DISTANCE;
	extern double IN_FRONT_ANGLE_DEG;
    extern double CAR_WIDTH;
    extern double CAR_LENGTH;

    extern double goalX;
    extern double goalZ;

    extern int MAX_EPISODE_LENGTH;

	const int N_PED_IN = 6;
	const int N_PED_WORLD = 200;

    extern double LASER_RANGE;

	extern double pos_rln; // position resolution
	extern double vel_rln; // velocity resolution

	extern double PATH_STEP;
	extern double GOAL_TOLERANCE;
	extern double PED_SPEED;

	extern int map_height;
	extern int map_width;

	extern bool debug;

	extern double control_freq;
	extern double AccSpeed;

	extern double GOAL_REWARD;

	//extern int numUpperBound ;
    //extern int numLowerBound ;
    //extern int numDefault ;

    extern SinCosLookupTable* lookupTable;
};

#endif

