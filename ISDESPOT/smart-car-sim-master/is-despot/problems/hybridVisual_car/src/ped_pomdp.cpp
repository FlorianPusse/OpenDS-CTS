#include "ped_pomdp.h"

class PedPomdpParticleLowerBound : public ParticleLowerBound {
private:
	const PedPomdp* ped_pomdp_;
public:
	PedPomdpParticleLowerBound(const DSPOMDP* model) :
		ParticleLowerBound(model),
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

    // IMPORTANT: Check after changing reward function.
	virtual ValuedAction Value(const vector<State*>& particles) const {
		PomdpState* state = static_cast<PomdpState*>(particles[0]);

		int min_step = numeric_limits<int>::max();
		auto& carpos = ped_pomdp_->world.path[state->car.pos];
		double carvel = state->car.vel;

        double min_dist = 100000;

		// Find mininum num of steps for car-pedestrian collision
		for (int i=0; i<state->num; i++) {
			auto& p = state->peds[i];

            // 3.25 is maximum distance to collision boundary from front laser (see collsion.cpp)
			double dist = max(COORD::EuclideanDistance(carpos, p.pos) - 3.25, 0.0);
			//if(ped_pomdp_->world.inFront(p.pos, state->car.pos)){
			    min_dist = min(dist, min_dist);
                int step = int(ceil(ModelParams::control_freq
                			* dist / ((p.vel + carvel))));
                min_step = min(step, min_step);
			//}
		}

        double move_penalty = ped_pomdp_->MovementPenalty(*state);

        // Case 1, no pedestrian: Constant car speed
		double value = move_penalty / (1 - Discount());

        // Case 2, with pedestrians: Constant car speed, head-on collision with nearest neighbor
		if (min_step != numeric_limits<int>::max()) {
		    double carvel = state->car.vel + 0.3;
            //double brakingDistance = ((carvel*3.6)*(carvel*3.6)) / (250*0.8);
            //if(brakingDistance >= min_dist){
                double crash_penalty = ped_pomdp_->CrashPenalty(*state);
                value = (move_penalty) * (1 - Discount(min_step)) / (1 - Discount())
                    + crash_penalty * Discount(min_step);
            //}
		}

		return ValuedAction(ped_pomdp_->ACT_CUR, State::Weight(particles) * value);
	}
};

PedPomdp::PedPomdp(WorldModel &model_) :
	world(model_),
	random_(Random((unsigned) Seeds::Next()))
{
	//particle_lower_bound_ = new PedPomdpParticleLowerBound(this);
}

vector<int> observation_vectors[3];
uint64_t PedPomdp::Observe(const State& state, int thread_number) const {
    const PomdpState &state_ = static_cast<const PomdpState&>(state);

    vector<int>& obs_vec = observation_vectors[thread_number];
    obs_vec.resize(state_.num * 2 + 2);

    obs_vec[0] = state_.car.pos;
    obs_vec[1] = int(state_.car.vel / ModelParams::vel_rln);

    int i = 2;

    for(int j = 0; j < state_.num; j ++) {
    	obs_vec[i++] = int(state_.peds[j].pos.x / ModelParams::pos_rln);
    	obs_vec[i++] = int(state_.peds[j].pos.y / ModelParams::pos_rln);
    }

	hash<vector<int>> myhash;
	return myhash(obs_vec);
}

uint64_t PedPomdp::Observe(const State& state) const {
    const PomdpState &state_ = static_cast<const PomdpState&>(state);

    vector<int> obs_vec;
    obs_vec.resize(state_.num * 2 + 2);

    obs_vec[0] = state_.car.pos;
    obs_vec[1] = int(state_.car.vel / ModelParams::vel_rln);

    int i = 2;

    for(int j = 0; j < state_.num; j ++) {
    	obs_vec[i++] = int(state_.peds[j].pos.x / ModelParams::pos_rln);
    	obs_vec[i++] = int(state_.peds[j].pos.y / ModelParams::pos_rln);
    }

	hash<vector<int>> myhash;
	return myhash(obs_vec);
}

vector<State*> PedPomdp::ConstructParticles(vector<PomdpState> & samples) {
	int num_particles=samples.size();
	vector<State*> particles;
	for(int i=0;i<samples.size();i++) {
		PomdpState* particle = static_cast<PomdpState*>(Allocate(-1, 1.0/num_particles));
		(*particle) = samples[i];
		particle->SetAllocated();
		particle->weight = 1.0/num_particles;
		particles.push_back(particle);
	}
	return particles;
}


// Very high cost for collision
double PedPomdp::CrashPenalty(const PomdpState& state) const { // , int closest_ped, double closest_dist) const {
	// double ped_vel = state.ped[closest_ped].vel;
    return -0.2 + ModelParams::CRASH_PENALTY * pow(0.5 + state.car.vel / ModelParams::VEL_MAX, 1.4);
}
// Very high cost for collision
double PedPomdp::CrashPenalty(const PomdpStateWorld& state) const { // , int closest_ped, double closest_dist) const {
	// double ped_vel = state.ped[closest_ped].vel;
    return -0.2 + ModelParams::CRASH_PENALTY * pow(0.5 + state.car.vel / ModelParams::VEL_MAX, 1.4);
}

// Avoid frequent dec or acc
double PedPomdp::ActionPenalty(int action) const {
    if(action == ACT_DEC || ACT_ACC){
        return -0.01;
    }

    return 0;
}

// Less penalty for longer distance travelled
double PedPomdp::MovementPenalty(const PomdpState& state) const {
    return (ModelParams::REWARD_FACTOR_VEL * (state.car.vel - ModelParams::VEL_MAX) / ModelParams::VEL_MAX) / 100.0;
}
// Less penalty for longer distance travelled
double PedPomdp::MovementPenalty(const PomdpStateWorld& state) const {
    return (ModelParams::REWARD_FACTOR_VEL * (state.car.vel - ModelParams::VEL_MAX) / ModelParams::VEL_MAX) / 100.0;
}

bool PedPomdp::Step(State& state_, double random_num, int action, double& reward,
	OBS_TYPE& obs, int thread_number) const {

	PomdpState& state = static_cast<PomdpState&>(state_);
	reward = 0.0;


	// CHECK: relative weights of each reward component
	// Terminate upon reaching goal

	if (world.isGlobalGoal(state.car) || state.car.dist_travelled >= 25) {
        reward = ModelParams::GOAL_REWARD;
		return true;
	}

 	// Safety control: collision; Terminate upon collision
    if(state.car.vel > 0.05 && world.inCollision(state) ) {
		reward = CrashPenalty(state); //, closest_ped, closest_dist);
		//cout << "Sample crash state: " << reward << "\n";
		return true;
	}

	// Forbidden actions
    double carvel = state.car.vel;
	/*if (action == ACT_CUR && 0.1 < carvel && carvel < 0.6) {
		reward = CrashPenalty(state);
		return true;
	}*/
    if (action == ACT_ACC && carvel >= ModelParams::VEL_MAX) {
		reward = CrashPenalty(state);
		return true;
    }
    if (action == ACT_DEC && carvel <= 0.01) {
		reward = CrashPenalty(state);
		return true;
    }

	// Smoothness control
	reward += ActionPenalty(action);

	// Speed control: Encourage higher speed
	reward += MovementPenalty(state);

	// State transition
	Random random(random_num);
	double acc = (action == ACT_ACC) ? ModelParams::AccSpeed : ((action == ACT_CUR) ?  0 : (-ModelParams::AccSpeed));
	world.RobStep(state.car, random);
	world.RobVelStep(state.car, acc, random);
	for(int i=0;i<state.num;i++)
		world.PedStep(state.peds[i], random);

	// Observation
	obs = Observe(state, thread_number);

	return false;
}

bool PedPomdp::Step(State& state_, double rNum, int action, double& reward, uint64_t& obs) const {
	PomdpState& state = static_cast<PomdpState&>(state_);
	reward = 0.0;

	// CHECK: relative weights of each reward component
	// Terminate upon reaching goal

	if (world.isGlobalGoal(state.car) || state.car.dist_travelled >= 25) {
        reward = ModelParams::GOAL_REWARD;
		return true;
	}

 	// Safety control: collision; Terminate upon collision
    if(state.car.vel > 0.01 && world.inCollision(state) ) {
		reward = CrashPenalty(state); //, closest_ped, closest_dist);
		//cout << "Sample crash state: " << reward << "\n";
		return true;
	}

	// Forbidden actions
    double carvel = state.car.vel;
	/*if (action == ACT_CUR && 0.1 < carvel && carvel < 0.6) {
		reward = CrashPenalty(state);
		return true;
	}*/
    if (action == ACT_ACC && carvel >= ModelParams::VEL_MAX) {
		reward = CrashPenalty(state);
		return true;
    }
    if (action == ACT_DEC && carvel <= 0.01) {
		reward = CrashPenalty(state);
		return true;
    }

	// Smoothness control
	reward += ActionPenalty(action);

	// Speed control: Encourage higher speed
	reward += MovementPenalty(state);

	// State transition
	Random random(rNum);
	double acc = (action == ACT_ACC) ? ModelParams::AccSpeed : ((action == ACT_CUR) ?  0 : (-ModelParams::AccSpeed));
	world.RobVelStep(state.car, acc, random);
	world.RobStep(state.car, random);
	for(int i=0;i<state.num;i++)
		world.PedStep(state.peds[i], random);

	// Observation
	obs = Observe(state);

	return false;
}

bool PedPomdp::Step(PomdpStateWorld& state, double rNum, int action, double& reward, uint64_t& obs) const {
	reward = 0.0;

	if (world.isGlobalGoal(state.car) || state.car.dist_travelled >= 25) {
        reward = ModelParams::GOAL_REWARD;
		return true;
	}

    if(state.car.vel > 0.01 && world.inCollision(state) ) {
		reward = CrashPenalty(state); //, closest_ped, closest_dist);
		return true;
	}

	// Forbidden actions
    double carvel = state.car.vel;
	/*if (action == ACT_CUR && 0.1 < carvel && carvel < 0.6) {
		reward = CrashPenalty(state);
		return true;
	}*/
    if (action == ACT_ACC && carvel >= ModelParams::VEL_MAX) {
		reward = CrashPenalty(state);
		return true;
    }
    if (action == ACT_DEC && carvel <= 0.01) {
		reward = CrashPenalty(state);
		return true;
    } 

	// Smoothness control
	reward += ActionPenalty(action);

	// Speed control: Encourage higher speed
	reward += MovementPenalty(state);

	// State transition
	Random random(rNum);
	double acc = (action == ACT_ACC) ? ModelParams::AccSpeed :((action == ACT_CUR) ?  0 : (-ModelParams::AccSpeed));
	//TODO: CHANGE ORDER?
	world.RobVelStep(state.car, acc, random);
	world.RobStep(state.car, random);
	for(int i=0;i<state.num;i++)
		world.PedStep(state.peds[i], random);



	return false;
}

bool PedPomdp::ImportanceSamplingStep(State& state_, double rNum, int action, double& reward, uint64_t& obs) const {
	PomdpState& state = static_cast<PomdpState&>(state_);
	reward = 0.0;

	if (world.isGlobalGoal(state.car) || state.car.dist_travelled >= 25) {
        reward = ModelParams::GOAL_REWARD;
		return true;
	}

 	// Safety control: collision; Terminate upon collision
	//if (closest_front_dist < ModelParams::COLLISION_DISTANCE) {
    if(state.car.vel > 0.01 && world.inCollision(state) ) { /// collision occurs only when car is moving
		reward = CrashPenalty(state); //, closest_ped, closest_dist);
		//cout << "Sample crash state: " << reward << "\n";
		return true;
	}

	// Forbidden actions

    double carvel = state.car.vel;
	/*if (action == ACT_CUR && 0.1 < carvel && carvel < 0.6) {
		reward = CrashPenalty(state);
		return true;
	}*/
    if (action == ACT_ACC && carvel >= ModelParams::VEL_MAX) {
		reward = CrashPenalty(state);
		return true;
    }
    if (action == ACT_DEC && carvel <= 0.01) {
		reward = CrashPenalty(state);
		return true;
    }

	// Smoothness control
	reward += ActionPenalty(action);

	// Speed control: Encourage higher speed
	reward += MovementPenalty(state);

	// State transition
	Random random(rNum);
	double acc = (action == ACT_ACC) ? ModelParams::AccSpeed : ((action == ACT_CUR) ?  0 : (-ModelParams::AccSpeed)); 
	world.RobStep(state.car, random);

	state.weight *= world.ISRobVelStep(state.car, acc, random);
	//world.RobVelStep(state.car, acc, random);
	
	for(int i=0;i<state.num;i++)
		//world.PedStep(state.peds[i], random);
		state.weight *= world.ISPedStep(state.car, state.peds[i], random);

	// Observation
	obs = Observe(state);



	return false;
}

double PedPomdp::ObsProb(uint64_t obs, const State& s, int action) const {
	return obs == Observe(s);
}

vector<vector<double>> PedPomdp::GetBeliefVector(const vector<State*> particles) const {
	vector<vector<double>> belief_vec;
	return belief_vec;
}

Belief* PedPomdp::InitialBelief(const State* start, string type) const {
	assert(false);
	return NULL;
}

/// output the probability of the intentions of the pedestrians
void PedPomdp::Statistics(const vector<PomdpState*> particles) const {
	/*double goal_count[10][10]={{0}};
	cout << "Current Belief" << endl;
	if(particles.size() == 0)
		return;

	PrintState(*particles[0]);
	PomdpState* state_0 = particles[0];
	for(int i = 0; i < particles.size(); i ++) {
		PomdpState* state = particles[i];
		for(int j = 0; j < state->num; j ++) {
			goal_count[j][state->peds[j].goal] += particles[i]->weight;
		}
	}

	for(int j = 0; j < state_0->num; j ++) {
		cout << "Ped " << j << " Belief is ";
		for(int i = 0; i < world.goals.size(); i ++) {
			cout << (goal_count[j][i] + 0.0) <<" ";
		}
		cout << endl;
	}*/
}

/// output the probability of the intentions of the pedestrians
void PedPomdp::PrintParticles(const vector<State*> particles, ostream& out) const {
	/*
	cout<<"******** scenario belief********"<<endl;

	cout << "Current Belief" << endl;
	if(particles.size() == 0)
		return;
	const PomdpState* pomdp_state=static_cast<const PomdpState*>(particles.at(0));

	auto goal_count = new double*[pomdp_state->num];
	for(int i = 0; i < pomdp_state->num; ++i){
		goal_count[i] = new double[world.goals.size()];
	}

	for(int i = 0; i < particles.size(); i ++) {
		const PomdpState* pomdp_state=static_cast<const PomdpState*>(particles.at(i));
		for(int j = 0; j < pomdp_state->num; j ++) {
			assert(pomdp_state->peds[j].id < ModelParams::N_PED_WORLD);
			goal_count[j][pomdp_state->peds[j].goal] += particles[i]->weight;
		}
	}

	for(int j = 0; j < pomdp_state->num; j ++) {
		cout << "Ped " << pomdp_state->peds[j].id << " Belief is ";
		assert(pomdp_state->peds[j].id < ModelParams::N_PED_WORLD);
		for(int i = 0; i < world.goals.size(); i ++) {
			cout << (goal_count[j][i] + 0.0) <<" ";
		}
		cout << endl;
	}

	for(int i = 0; i < pomdp_state->num; ++i){
		delete[] goal_count[i];
	}

	delete[] goal_count;

	cout<<"******** end of scenario belief********"<<endl;
	*/
}

ValuedAction PedPomdp::GetMinRewardAction() const {
	return ValuedAction(0, 
			ModelParams::CRASH_PENALTY * (ModelParams::VEL_MAX*ModelParams::VEL_MAX + ModelParams::REWARD_BASE_CRASH_VEL));
}

class PedPomdpSmartScenarioLowerBound : public Policy {
protected:
	const PedPomdp* ped_pomdp_;

public:
	PedPomdpSmartScenarioLowerBound(const DSPOMDP* model, ParticleLowerBound* bound) :
		Policy(model,bound),
		//ped_pomdp_(model)
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const {
		return ped_pomdp_->world.defaultPolicy(particles);
	}
};

ScenarioLowerBound* PedPomdp::CreateScenarioLowerBound(string name,
		string particle_bound_name) const {

	// name = "TRIVIAL";
	name="SMART";
	if(name == "TRIVIAL") {
		return new TrivialParticleLowerBound(this);
	} else if(name == "RANDOM") {
		return new RandomPolicy(this, new PedPomdpParticleLowerBound(this));
	} else if (name == "SMART") {
		return new PedPomdpSmartScenarioLowerBound(this, new PedPomdpParticleLowerBound(this));
	} else {
		cerr << "Unsupported scenario lower bound: " << name << endl;
		exit(0);
	}
}

double PedPomdp::GetMaxReward() const {
	return 0;
}

class PedPomdpSmartParticleUpperBound : public ParticleUpperBound {
protected:
	const PedPomdp* ped_pomdp_;
public:
	PedPomdpSmartParticleUpperBound(const DSPOMDP* model) :
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

    // IMPORTANT: Check after changing reward function.
	double Value(const State& s) const {
	    // ModelParams::numUpperBound++;

		const PomdpState& state = static_cast<const PomdpState&>(s);

        if (ped_pomdp_->world.inCollision(state))
            return ped_pomdp_->CrashPenalty(state);

        int min_step = ped_pomdp_->world.minStepToGoal(state);

        double val = ModelParams::GOAL_REWARD * Discount(min_step) + 0.2;

		return val;
	}
};

ParticleUpperBound* PedPomdp::CreateParticleUpperBound(string name) const{
	name = "SMART";
	if (name == "TRIVIAL") {
		return new TrivialParticleUpperBound(this);
	} else if (name == "SMART") {
		return new PedPomdpSmartParticleUpperBound(this);
	} else {
		cerr << "Unsupported particle upper bound: " << name << endl;
		exit(0);
	}
}

ScenarioUpperBound* PedPomdp::CreateScenarioUpperBound(string name,
		string particle_bound_name) const{
	name = "SMART";
	if (name == "TRIVIAL") {
		return new TrivialParticleUpperBound(this);
	} else if (name == "SMART") {
		return new PedPomdpSmartParticleUpperBound(this);
	}
	else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(0);
	}
}

void PedPomdp::PrintState(const State& s, ostream& out) const {
	const PomdpState & state=static_cast<const PomdpState&> (s);
    COORD& carpos = world.path[state.car.pos];

	out << "car pos / dist_trav / vel = " << "(" << carpos.x<< ", " <<carpos.y << ", " << carpos.theta <<  ") / "
        << state.car.dist_travelled << " / "
        << state.car.vel << endl;
	out<< state.num << " pedestrians " << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "ped " << i << ": id / pos / vel / goal / dist2car / infront =  " << state.peds[i].id << " / "
            << "(" << state.peds[i].pos.x << ", " << state.peds[i].pos.y << ") / "
            << state.peds[i].vel << " / "
            << state.peds[i].goal << " / "
            << COORD::EuclideanDistance(state.peds[i].pos, carpos) << "/"
			<< world.inFront(state.peds[i].pos, state.car.pos) << endl;
	}
    double min_dist = -1;
    if (state.num > 0){
        min_dist = COORD::EuclideanDistance(carpos, state.peds[0].pos);
		out << "MinDist: " << min_dist << endl;
	}
}

void PedPomdp::PrintObs(const State&state, uint64_t obs, ostream& out) const {
	out << obs << endl;
}

void PedPomdp::PrintAction(int action, ostream& out) const {
	out << action << endl;
}

void PedPomdp::PrintBelief(const Belief& belief, ostream& out ) const {
	
}

State* PedPomdp::Allocate(int state_id, double weight) const {
	//num_active_particles ++;
	PomdpState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State* PedPomdp::Copy(const State* particle) const {
	//num_active_particles ++;
	PomdpState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const PomdpState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void PedPomdp::Free(State* particle) const {
	//num_active_particles --;
	memory_pool_.Free(static_cast<PomdpState*>(particle));
}


int PedPomdp::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}
