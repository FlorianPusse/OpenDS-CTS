#include "core/pomdp.h"
#include "core/policy.h"
#include "core/lower_bound.h"
#include "core/upper_bound.h"
#include "solver/pomcp.h"

/* =============================================================================
 * State class
 * =============================================================================*/

ostream& operator<<(ostream& os, const State& state) {
	os << "(state_id = " << state.state_id << ", weight = " << state.weight
		<< ", text = " << (&state)->text() << ")";
	return os;
}

State::State() :
	state_id(-1) {
}

State::State(int _state_id, double _weight) :
	state_id(_state_id),
	weight(_weight) {
}

State::~State() {
}

string State::text() const {
	return "AbstractState";
}

double State::Weight(const vector<State*>& particles) {
	double weight = 0;
	for (int i = 0; i < particles.size(); i++)
		weight += particles[i]->weight;
	return weight;
}
/* =============================================================================
 * StateIndexer class
 * =============================================================================*/
StateIndexer::~StateIndexer() {
}

/* =============================================================================
 * StatePolicy class
 * =============================================================================*/
StatePolicy::~StatePolicy() {
}

/* =============================================================================
 * MMAPinferencer class
 * =============================================================================*/
MMAPInferencer::~MMAPInferencer() {
}

/* =============================================================================
 * DSPOMDP class
 * =============================================================================*/

DSPOMDP::DSPOMDP() {
}

DSPOMDP::~DSPOMDP() {
}

bool DSPOMDP::Step(State& state, int action, double& reward,
	OBS_TYPE& obs) const {
	return Step(state, Random::RANDOM.NextDouble(), action, reward, obs);
}

bool DSPOMDP::Step(State& state, double random_num, int action,
	double& reward) const {
	OBS_TYPE obs;
	return Step(state, random_num, action, reward, obs);
}

bool DSPOMDP::ImportanceSamplingStep(State& state, double random_num, int action,
	double& reward, OBS_TYPE& obs) const{
	//some tasks only do importance sampling for initial belief. Calling Step() does not 
	//necessarily mean it did not use importance sampling
	return Step(state, random_num, action, reward, obs);
}


ParticleUpperBound* DSPOMDP::CreateParticleUpperBound(string name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialParticleUpperBound(this);
	} else {
		cerr << "Unsupported particle upper bound: " << name << endl;
		exit(1);
	}
}

ScenarioUpperBound* DSPOMDP::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialParticleUpperBound(this);
	} else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

ParticleLowerBound* DSPOMDP::CreateParticleLowerBound(string name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialParticleLowerBound(this);
	} else {
		cerr << "Unsupported particle lower bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

ScenarioLowerBound* DSPOMDP::CreateScenarioLowerBound(string name, string
	particle_bound_name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialParticleLowerBound(this);
	} else if (name == "RANDOM") {
		return new RandomPolicy(this, CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(1);
		return NULL;
	}
}

POMCPPrior* DSPOMDP::CreatePOMCPPrior(string name) const {
	if (name == "UNIFORM" || name == "DEFAULT") {
		return new UniformPOMCPPrior(this);
	} else {
		cerr << "Unsupported POMCP prior: " << name << endl;
		exit(1);
		return NULL;
	}
}

vector<State*> DSPOMDP::Copy(const vector<State*>& particles) const {
	vector<State*> copy;
	for (int i = 0; i < particles.size(); i++)
		copy.push_back(Copy(particles[i]));
	return copy;
}


vector<double> DSPOMDP::ImportanceWeight(vector<State*> particles) const{
	vector <double> importance_weight;
	for(int i=0; i<particles.size();i++){
		importance_weight.push_back(particles[i]->weight);
	}

	return importance_weight;
}

void DSPOMDP::PrintParticles(const vector<State*> particles, ostream& out) const{
}

vector<double> DSPOMDP::Feature(const State& state) const{
	vector<double> v;
	return v;
}

/* =============================================================================
 * BeliefMDP classs
 * =============================================================================*/

BeliefMDP::BeliefMDP() {
}

BeliefMDP::~BeliefMDP() {
}

BeliefLowerBound* BeliefMDP::CreateBeliefLowerBound(string name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialBeliefLowerBound(this);
	} else {
		cerr << "Unsupported belief lower bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

BeliefUpperBound* BeliefMDP::CreateBeliefUpperBound(string name) const {
	if (name == "TRIVIAL" || name == "DEFAULT") {
		return new TrivialBeliefUpperBound(this);
	} else {
		cerr << "Unsupported belief upper bound: " << name << endl;
		exit(1);
		return NULL;
	}
}

