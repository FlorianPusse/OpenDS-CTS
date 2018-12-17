#include<limits>
#include<cmath>
#include<cstdlib>
#include"WorldModel.h"
#include"math_utils.h"
#include"coord.h"
#include<numeric>
#include <fstream>
using namespace std;

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

WorldModel::WorldModel(): freq(ModelParams::control_freq),
    in_front_angle_cos(cos(ModelParams::IN_FRONT_ANGLE_DEG / 180.0 * M_PI)) {

    /*
    std::ifstream infile("goals");

    int x, z;
    while (infile >> x >> z)
    {
        goals.push_back(COORD(x,z));
    }*/

    /*
    goals.push_back(COORD(-1,-1));*/   // stop intention

    for(int i = 0; i < 360; i+= ModelParams::BELIEF_ANGLE_DISCRETIZATION){
        goals.push_back(((double) i) * (M_PI / 180.0));
    }

    goals.push_back(-1);
}

bool WorldModel::isGlobalGoal(const CarStruct& car) {
    double d = COORD::EuclideanDistanceSimple(path[car.pos], path[path.size()-1]);
    return (d<(ModelParams::GOAL_TOLERANCE*ModelParams::GOAL_TOLERANCE));
}

enum {
	ACT_CUR,
	ACT_ACC,
	ACT_DEC
};

int WorldModel::defaultPolicy(const vector<State*>& particles)  {
	//ModelParams::numDefault++;

	const PomdpState *state=static_cast<const PomdpState*>(particles[0]);

        double mindist = numeric_limits<double>::infinity();
        auto& carpos = path[state->car.pos];
        double carvel = state->car.vel + 1.5;

        double mindist_all = numeric_limits<double>::infinity();

        // Closest pedestrian in front
        for (int i=0; i<state->num; i++) {
    	    auto& p = state->peds[i];

    	    double d = COORD::EuclideanDistance(carpos, p.pos);
            if (d >= 0 && d < mindist_all)
                mindist_all = d;

    	    if(!WorldModel::inFront(p.pos, state->car.pos))
                continue;

            d = COORD::EuclideanDistance(carpos, p.pos);
            if (d >= 0 && d < mindist)
    			mindist = d;
        }

    	double brakingDistance = ((carvel*3.6)*(carvel*3.6)) / (250*0.8);

        if(state->car.vel > 0.1 && (brakingDistance > mindist || mindist_all < 2)){
            return ACT_DEC;
        }

        double nextBrakingDistance = (((carvel+ModelParams::AccSpeed)*3.6)*((carvel+ModelParams::AccSpeed)*3.6)) / (250*0.8);
        if((state->car.vel + 0.1 < ModelParams::VEL_MAX) && (nextBrakingDistance < mindist)){
            return ACT_ACC;
        }

        return ACT_CUR;
}

bool WorldModel::inFront(COORD ped_pos, int car) const {
    if(ModelParams::IN_FRONT_ANGLE_DEG >= 180.0) {
        // inFront check is disabled
        return true;
    }

	const COORD& car_pos = path[car];
	const COORD& forward_pos = path[path.forward(car, 1.0)];

	double d0 = COORD::EuclideanDistance(car_pos, ped_pos);
	if(d0 <= 0.7) return true;

	double d1 = COORD::EuclideanDistance(car_pos, forward_pos);
	if(d1<=0) return false;

	double dot = DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
			ped_pos.x - car_pos.x, ped_pos.y - car_pos.y);
	double cosa = dot / (d0 * d1);

    if(!(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8)){
        cout << "\nCosa value: " << cosa << "\n";
        cout << "Dot value: " << dot << "\n";
        cout << "carpos value: " << car_pos.x << ", " << car_pos.y << "\n";
        cout << "pedpos value: " << ped_pos.x << ", " << ped_pos.y << "\n";
        cout << "d0 value: " << d0 << "\n";
        cout << "d1 value: " << d1 << "\n";
        assert(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8);
    }

    return cosa > in_front_angle_cos;
}

/**
 * H: center of the head of the car
 * N: a point right in front of the car
 * M: an arbitrary point
 *
 * Check whether M is in the safety zone
 */
//bool inCollision(double Mx, double My, double Hx, double Hy, double Nx, double Ny);
bool inCollision(const double Mx, const double My, const COORD& car_pos);

bool WorldModel::inCollision(const PomdpState& state) {
    const int car = state.car.pos;
	const COORD& car_pos = path[car];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;

        // COORD::EuclideanDistance(pedpos,car_pos) >= 5
        if(COORD::EuclideanDistanceSimple(pedpos,car_pos) >= 25){
            continue;
        } 

        if(::inCollision(pedpos.x, pedpos.y, car_pos)) {
            return true;
        }
    }
    return false;
}

/**
 * Checks whether a collision occurs in the current state.
 * @param state the current state
 * @return true, if a collision occurs, else otherwise
 */
bool WorldModel::inCollision(const PomdpStateWorld& state) {
    const int car = state.car.pos;
    const COORD& car_pos = path[car];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;

        if(COORD::EuclideanDistanceSimple(pedpos,car_pos) >= 25){
            continue;
        } 

        if(::inCollision(pedpos.x, pedpos.y, car_pos)) {
            return true;
        }
    }
    return false;
}

/**
 * Checks whether a collision occurs in the current LOCAL state. If a collusion occurs,
 * the id of ONE pedestrian is stored in @param id
 * @param state the current state
 * @param id the id of ONE participant that the car collides with, -1 if no collision
 * @return true, if a collision occurs, else otherwise
 */
bool WorldModel::inCollision(const PomdpState& state, int &id) {
	id=-1;
    const int car = state.car.pos;
	const COORD& car_pos = path[car];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;

        if(COORD::EuclideanDistanceSimple(pedpos,car_pos) >= 25){
            continue;
        } 

        if(::inCollision(pedpos.x, pedpos.y, car_pos)) {
        	id=state.peds[i].id;
            return true;
        }
    }
    return false;
}

/**
 * Checks whether a collision occurs in the current WORLD state. If a collusion occurs,
 * the id of ONE pedestrian is stored in @param id
 * @param state the current state
 * @param id the id of ONE participant that the car collides with, -1 if no collision
 * @return true, if a collision occurs, else otherwise
 */
bool WorldModel::inCollision(const PomdpStateWorld& state, int &id) {
    id=-1;
    const int car = state.car.pos;
    const COORD& car_pos = path[car];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;

        if(COORD::EuclideanDistanceSimple(pedpos,car_pos) >= 25){
            continue;
        } 

        if(::inCollision(pedpos.x, pedpos.y, car_pos)) {
        	id=state.peds[i].id;
            return true;
        }
    }
    return false;
}


void WorldModel::getClosestPed(const PomdpState& state, 
		int& closest_front_ped,
		double& closest_front_dist,
		int& closest_side_ped,
		double& closest_side_dist) {
	closest_front_ped = -1;
	closest_front_dist = numeric_limits<double>::infinity();
	closest_side_ped = -1;
	closest_side_dist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		bool front = inFront(p.pos, state.car.pos);
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (front) {
			if (d < closest_front_dist) {
				closest_front_dist = d;
				closest_front_ped = i;
			}
		} else {
			if (d < closest_side_dist) {
				closest_side_dist = d;
				closest_side_ped = i;
			}
		}
    }
}

bool WorldModel::isMovingAway(const PomdpState& state, int ped) {
    const auto& carpos = path[state.car.pos];
	const auto& nextcarpos = path[path.forward(state.car.pos, 1.0)];

	const auto& pedpos = state.peds[ped].pos;
	double goalAngle = goals[state.peds[ped].goal];
	MyVector goal_vec(ModelParams::lookupTable->cos(goalAngle),ModelParams::lookupTable->sin(goalAngle));
    COORD goalpos(pedpos.x + goal_vec.dw, pedpos.y + goal_vec.dh);

	if (goalAngle == -1)
		return false;

	return DotProduct(goalpos.x - pedpos.x, goalpos.y - pedpos.y,
			nextcarpos.x - carpos.x, nextcarpos.y - carpos.y) > 0;
}

///get the min distance between car and the peds in its front
double WorldModel::getMinCarPedDist(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		if(!inFront(p.pos, state.car.pos)) continue;
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

///get the min distance between car and the peds
double WorldModel::getMinCarPedDistAllDirs(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

int WorldModel::minStepToGoal(const PomdpState& state) {
    double d = COORD::EuclideanDistance(path[state.car.pos], path[path.size()-1]);
    if (d < 0) d = 0;
    return int(ceil(d / (ModelParams::VEL_MAX/freq)));
}

void WorldModel::PedStep(PedStruct &ped, Random& random) {
    const double& goal = goals[ped.goal];
	if (goal == -1) {  //stop intention
		return;
	}

    double a = goal;
	double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
    a += noise;

	//TODO noisy speed
    MyVector move(a, ped.vel/freq, 0);

    if((std::isnan(move.dw)) || (std::isnan(move.dh))){
        cout << "a: " << a << "\n";
        cout << "ped.vel/freq" << (ped.vel/freq) << "\n";
        double dw=(ped.vel/freq)*ModelParams::lookupTable->cos(a);
        double dh=(ped.vel/freq)*ModelParams::lookupTable->sin(a);
        cout << "sin: " << ModelParams::lookupTable->sin(a) << "\n";
        cout << "cos: " << ModelParams::lookupTable->cos(a) << "\n";
        cout << "dw: " << dw << "\n";
        cout << "dh: " << dh << "\n";
        cout << "dw nan: " << dw << "\n";
        cout << "dh nan: " << dh << "\n";
        assert((!std::isnan(move.dw)) && (!std::isnan(move.dh)));
    }

    ped.pos.x += move.dw;
    ped.pos.y += move.dh;
    return;
}

double gaussian_prob(double x, double stddev) {
    double a = 1.0 / stddev / sqrt(2 * M_PI);
    double b = - x * x / 2.0 / (stddev * stddev);
    return a * exp(b);
}

double WorldModel::ISPedStep(CarStruct &car, PedStruct &ped, Random& random) {
    //gaussian + distance
    double max_is_angle = 7.0*M_PI/64.0;
    COORD carpos = path[car.pos];
    if(COORD::EuclideanDistance(ped.pos, carpos)>3.5){
        const double& goal = goals[ped.goal];
        if (goal == -1) {  //stop intention
            return 1;
        }

        double a = goal;
        double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
        a += noise;

        MyVector move(a, ped.vel/freq, 0);

        ped.pos.x += move.dw;
        ped.pos.y += move.dh;
        return 1;
    } else{
        double weight = 1.0;
        const double& goal = goals[ped.goal];
        if (goal == -1) {  //stop intention
            return weight;
        }

        double goal_angle = goal;

        //compute the angle to robot
        MyVector rob_vec(path[car.pos].x - ped.pos.x, path[car.pos].y - ped.pos.y);
        double rob_angle = rob_vec.GetAngle();

        double final_mean; //final mean angle

        if(abs(goal_angle-rob_angle) <= M_PI){
            if(goal_angle > rob_angle) final_mean = goal_angle - min(max_is_angle, goal_angle-rob_angle);
            else  final_mean = goal_angle + min(max_is_angle, rob_angle-goal_angle);
        }
        else{
            if(goal_angle > rob_angle) final_mean = goal_angle + min(max_is_angle, rob_angle+2*M_PI-goal_angle);
            else  final_mean = goal_angle - min(max_is_angle, goal_angle+2*M_PI-rob_angle);
        }

        if(final_mean>M_PI) final_mean -= M_PI;
        else if(final_mean<-M_PI) final_mean += M_PI;

        //random.NextGaussian() returns a random number sampled from N(0,1)
        double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE; //change to the number sampled from N(0, ModelParams::NOISE_GOAL_ANGLE)
        double final_angle = final_mean + noise; //change to the number sampled from N(rob_angle, ModelParams::NOISE_GOAL_ANGLE)

        //TODO noisy speed
        MyVector move(final_angle, ped.vel/freq, 0);
        ped.pos.x += move.dw;
        ped.pos.y += move.dh;

        weight = gaussian_prob((final_angle - goal_angle) / ModelParams::NOISE_GOAL_ANGLE, 1) /
                 gaussian_prob((final_angle - final_mean) / ModelParams::NOISE_GOAL_ANGLE, 1) ;
        return weight;
    }
}

void WorldModel::PedStepDeterministic(PedStruct& ped, int step) {
    const double& goal = goals[ped.goal];
	if (goal == -1) {  //stop intention
		return;
	}

	MyVector goal_vec(ModelParams::lookupTable->cos(goal),ModelParams::lookupTable->sin(goal));
    goal_vec.AdjustLength(step * ped.vel / freq);

    assert((!std::isnan(goal_vec.dw)) && (!std::isnan(goal_vec.dh)));

    ped.pos.x += goal_vec.dw;
    ped.pos.y += goal_vec.dh;
}

double WorldModel::pedMoveProb(COORD prev, COORD curr, int goal_id) {
	const double K = 0.001;
    const double& goal_ = goals[goal_id];

    MyVector goal_vec(ModelParams::lookupTable->cos(goal_),ModelParams::lookupTable->sin(goal_));
    goal_vec.AdjustLength(1);

	double move_dist = Norm(curr.x-prev.x, curr.y-prev.y);
	double sensor_noise = 0.1;

    bool debug=false;
	// CHECK: beneficial to add back noise?
	if(debug) cout<<"goal id "<<goal_id<<endl;


	if (goal_ == -1) {  //stop intention
    	return (move_dist < sensor_noise) ? 0.4 : 0;
    }else {
		if (move_dist < sensor_noise) return 0;

        MyVector ped_movement_direction(curr.x-prev.x, curr.y-prev.y);
        ped_movement_direction.AdjustLength(1);

		double cosa = DotProduct(ped_movement_direction.dw, ped_movement_direction.dh, goal_vec.dw, goal_vec.dh);
		double angle = acos(cosa);
		return gaussian_prob(angle, ModelParams::NOISE_GOAL_ANGLE) + K;
	}
}

void WorldModel::RobStep(CarStruct &car, Random& random) {
    double dist = car.vel / freq;

    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, Random& random) {
    const double N = ModelParams::NOISE_ROBVEL;
    if (N>0) {
        double prob = random.NextDouble();
        if (prob > N) {
            car.vel += acc; // / freq;
        }
    } else {
        car.vel += acc; // / freq;
    }

	car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);

	return;
}

double WorldModel::ISRobVelStep(CarStruct &car, double acc, Random& random) {
    const double N = 4 * ModelParams::NOISE_ROBVEL;
    double weight = 1;
    if (N>0) {
        double prob = random.NextDouble();
        if (prob > N) {
            car.vel += acc; // / freq;
            weight = (1.0 - ModelParams::NOISE_ROBVEL)/(1.0 -N);
        }
        else weight = ModelParams::NOISE_ROBVEL / N;
    } else {
        car.vel += acc; // / freq;
    }

    car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);

    return weight;
}

void WorldModel::setPath(Path path) {
    this->path = path;
}

void WorldModel::updatePedBelief(PedBelief& b, const PedStruct& curr_ped) {
    const double ALPHA = 0.8;
	const double SMOOTHING=ModelParams::BELIEF_SMOOTHING;

    bool debug=false;
	
    for(int i=0; i<goals.size(); i++) {
		double prob = pedMoveProb(b.pos, curr_ped.pos, i);
        b.prob_goals[i] *=  prob;

		// Important: Keep the belief noisy to avoid aggressive policies
		b.prob_goals[i] += SMOOTHING / goals.size(); // CHECK: decrease or increase noise
	}
	if(debug) {
        for(double w: b.prob_goals) {
            cout << w << " ";
        }
        cout << endl;
    }

    // normalize
    double total_weight = std::accumulate(b.prob_goals.begin(), b.prob_goals.end(), double(0.0));
	if(debug) cout << "total_weight = " << total_weight << endl;
    for(double& w : b.prob_goals) {
        w /= total_weight;
    }

    double moved_dist = COORD::EuclideanDistance(b.pos, curr_ped.pos);
    b.vel = ALPHA * b.vel + (1-ALPHA) * moved_dist * ModelParams::control_freq;

    if(b.vel > 15){
        b.vel = 1;
    }

	b.pos = curr_ped.pos;
}

PedBelief WorldModel::initPedBelief(const PedStruct& ped) {
    PedBelief b = {ped.id, ped.pos, ModelParams::PED_SPEED, vector<double>(goals.size(), 1.0/goals.size())};
    return b;
}

double timestamp() {
    //return ((double) clock()) / CLOCKS_PER_SEC;
    static double starttime=get_time_second();
    return get_time_second()-starttime;
}

void WorldStateTracker::cleanPed() {
    vector<Pedestrian> ped_list_new;
    for(int i=0;i<ped_list.size();i++)
    {
        bool insert=true;
        double w1,h1;
        w1=ped_list[i].x;
        h1=ped_list[i].y;
        for(const auto& ped: ped_list_new) {
            double w2,h2;
            w2=ped.x;
            h2=ped.y;
            if (abs(w1-w2)<=0.1&&abs(h1-h2)<=0.1) {
                insert=false;
                break;
            }
        }
        if (timestamp() - ped_list[i].last_update > 0.2) insert=false;
        if (insert)
            ped_list_new.push_back(ped_list[i]);
    }
    ped_list=ped_list_new;
}

void WorldStateTracker::updatePed(const Pedestrian& ped){
    int i=0;
    for(;i<ped_list.size();i++) {
        if (ped_list[i].id==ped.id) {
            ped_list[i].x=ped.x;
            ped_list[i].y=ped.y;
            ped_list[i].last_update = timestamp();
            break;
        }
    }
    if (i==ped_list.size()) {
        ped_list.push_back(ped);
        ped_list.back().last_update = timestamp();
    }
}

void WorldStateTracker::updateCar(const COORD& car) {
    carpos=car;
}

void WorldStateTracker::updateVel(double vel) {
	carvel = vel;
}

vector<WorldStateTracker::PedDistPair> WorldStateTracker::getSortedPeds() {
    vector<PedDistPair> sorted_peds;
    for(const auto& ped: ped_list) {
        COORD cp(ped.x, ped.y);
        float dist = COORD::EuclideanDistance(cp, carpos);
        sorted_peds.push_back(PedDistPair(dist, ped));
    }

    sort(sorted_peds.begin(), sorted_peds.end(),
            [](const PedDistPair& a, const PedDistPair& b) -> bool {
                return a.first < b.first;
            });

    return sorted_peds;
}

PomdpState WorldStateTracker::getPomdpState() {
    auto sorted_peds = getSortedPeds();

    // construct PomdpState
    PomdpState pomdpState;
    pomdpState.car.pos = model.path.nearest(carpos);
    pomdpState.car.vel = carvel;
	pomdpState.car.dist_travelled = 0;
    pomdpState.num = sorted_peds.size();

	if (pomdpState.num > ModelParams::N_PED_IN) {
		pomdpState.num = ModelParams::N_PED_IN;
	}

    cout<<"pedestrian time stamps"<<endl;
    for(int i=0;i<pomdpState.num;i++) {
        const auto& ped = sorted_peds[i].second;
        pomdpState.peds[i].pos.x=ped.x;
        pomdpState.peds[i].pos.y=ped.y;
		pomdpState.peds[i].id = ped.id;
        //pomdpState.peds[i].vel = ped.vel;
		pomdpState.peds[i].goal = -1;
        cout<<"ped "<<i<<" "<<ped.last_update<<endl;
    }
	return pomdpState;
}

void WorldBeliefTracker::update() {
    // update car
    car.pos = model.path.nearest(stateTracker.carpos);
    car.vel = stateTracker.carvel;
	car.dist_travelled = 0;

    auto sorted_peds = stateTracker.getSortedPeds();
    map<int, PedStruct> newpeds;
    for(const auto& dp: sorted_peds) {
        auto& p = dp.second;
        assert (p.id < ModelParams::N_PED_WORLD);
        PedStruct ped(COORD(p.x, p.y), -1, p.id);
        assert (ped.id < ModelParams::N_PED_WORLD);
        newpeds[p.id] = ped;
    }

    // remove disappeared peds
    vector<int> peds_to_remove;
    for(const auto& p: peds) {
        if (newpeds.find(p.first) == newpeds.end()) {
            peds_to_remove.push_back(p.first);
        }
    }
    for(const auto& i: peds_to_remove) {
        peds.erase(i);
    }

    // update existing peds
    for(auto& kv : peds) {
        PedStruct& curr_ped = newpeds[kv.first];
        model.updatePedBelief(kv.second, curr_ped);
    }

    // add new peds
    for(const auto& kv: newpeds) {
		auto& p = kv.second;
        assert (p.id < ModelParams::N_PED_WORLD);
        if (peds.find(p.id) == peds.end()) {
            peds[p.id] = model.initPedBelief(p);
        }
    }

	sorted_beliefs.clear();
	for(const auto& dp: sorted_peds) {
		auto& p = dp.second;
        assert (p.id < ModelParams::N_PED_WORLD);
        assert (peds[p.id].id < ModelParams::N_PED_WORLD);
		sorted_beliefs.push_back(peds[p.id]);
	}

    return;
}

int PedBelief::sample_goal() const {
    double r = double(rand()) / RAND_MAX;
    int i = 0;
    r -= prob_goals[i];
    while(r > 0) {
        i++;
        r -= prob_goals[i];
    }
    return i;
}

int PedBelief::maxlikely_goal() const {
    double ml = 0;
    int mi = prob_goals.size()-1; // stop intention
    for(int i=0; i<prob_goals.size(); i++) {
        if (prob_goals[i] > ml && prob_goals[i] > 0.5) {
            ml = prob_goals[i];
            mi = i;
        }
    }
    return mi;
}

void WorldBeliefTracker::printBelief() const {
	/*int num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < 100) {
            cout << "ped belief " << num << ": ";
            for (int g = 0; g < p.prob_goals.size(); g ++)
                cout << " " << p.prob_goals[g];
            cout << endl;
		}
    }*/
}

PomdpState WorldBeliefTracker::sample() {
    PomdpState s;
    s.car = car;

	s.num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < ModelParams::LASER_RANGE) {
			s.peds[s.num].pos = p.pos;
			s.peds[s.num].goal = p.sample_goal();
			s.peds[s.num].id = p.id;
            s.peds[s.num].vel = p.vel;
			s.num ++;
		}
    }

    return s;
}

vector<PomdpState> WorldBeliefTracker::sample(int num) {
    vector<PomdpState> particles;
    for(int i=0; i<num; i++) {
        particles.push_back(sample());
    }

    cout << "Num peds for planning: " << particles[0].num << endl;

    return particles;
}

void WorldBeliefTracker::PrintState(const State& s, ostream& out) const {
	const PomdpState & state=static_cast<const PomdpState&> (s);
    COORD& carpos = model.path[state.car.pos];

	out << "Rob Pos: " << carpos.x<< " " <<carpos.y << endl;
	out << "Rob travelled: " << state.car.dist_travelled << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "Ped Pos: " << state.peds[i].pos.x << " " << state.peds[i].pos.y << endl;
		out << "Goal: " << state.peds[i].goal << endl;
		out << "id: " << state.peds[i].id << endl;
	}
	out << "Vel: " << state.car.vel << endl;
	out<<  "num  " << state.num << endl;
	double min_dist = COORD::EuclideanDistance(carpos, state.peds[0].pos);
	out << "MinDist: " << min_dist << endl;
}

