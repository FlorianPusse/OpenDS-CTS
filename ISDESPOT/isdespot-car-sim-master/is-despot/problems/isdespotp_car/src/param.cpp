#include"param.h"

namespace ModelParams {
    int BELIEF_ANGLE_DISCRETIZATION = 2;

    double CRASH_PENALTY = -1.0;
    double REWARD_FACTOR_VEL = 0.5;
	double VEL_MAX = 50 * 0.27778;
	double NOISE_GOAL_ANGLE = 3.14*0.06;//3.14 * 0.25; //use 0 for debugging
    double REWARD_BASE_CRASH_VEL=0.5;
    double BELIEF_SMOOTHING = 0.05;
    double NOISE_ROBVEL = 0.05; //0; //0.1;
    double IN_FRONT_ANGLE_DEG = 70;

    double COLLISION_DISTANCE = 2.0;
    double COLLISION_SIDE_DISTANCE = 1.2;
    double CAR_WIDTH = 1.8;
    double CAR_LENGTH = 4.3;

	int MAX_EPISODE_LENGTH = 400;

	double LASER_RANGE = 50.0;

	double pos_rln = 0.25; // position resolution
	double vel_rln = 5 * 0.2778; // velocity resolution

	double PATH_STEP = 0.2;
	double GOAL_TOLERANCE = 2;
	double PED_SPEED = 1.2;

    int map_height = 305;
    int map_width = 387;

	bool debug = false;

	double control_freq = 4;
	double AccSpeed = 5 * 0.2778;

	double GOAL_REWARD = 1.0;

	//int numUpperBound = 0;
	//int numLowerBound = 0;
	//int numDefault = 0;

	SinCosLookupTable* lookupTable = new SinCosLookupTable();
}

