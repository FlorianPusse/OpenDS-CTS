#include "solver/despot.h"
#include "solver/pomcp.h"
#include <thread>

DESPOT::DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief) :
	Solver(model, belief),
	root_(NULL), 
	lower_bound_(lb),
	upper_bound_(ub) {
	assert(model != NULL);
}

DESPOT::~DESPOT() {
}

ScenarioLowerBound* DESPOT::lower_bound() const {
	return lower_bound_;
}

ScenarioUpperBound* DESPOT::upper_bound() const {
	return upper_bound_;
}

int DESPOT::call_counter = 0;

/*
 * CarStruct car;
 * int num;
 * PedStruct peds[ModelParams::N_PED_IN];
*/


VNode* DESPOT::Trial(VNode* root, RandomStreams& streams,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, History& history, SearchStatistics* statistics) {
	VNode* cur = root;

	int hist_size = history.Size();

	do {
		ExploitBlockers(cur);

		if (Gap(cur) == 0) {
			break;
		}

		if (cur->IsLeaf()) {
			Expand(cur, lower_bound, upper_bound, model, streams, history);
		}

		QNode* qstar = SelectBestUpperBoundNode(cur);
		VNode* next = SelectBestWEUNode(qstar);

		if (next == NULL) {
			break;
		}

		cur = next;
		history.Add(qstar->edge(), cur->edge());

	} while (cur->depth() < Globals::config.search_depth && WEU(cur) > 0);

	history.Truncate(hist_size);

	return cur;
}

void DESPOT::ExploitBlockers(VNode* vnode) {
	if (Globals::config.pruning_constant <= 0) {
		return;
	}

	VNode* cur = vnode;
	while (cur != NULL) {
		VNode* blocker = FindBlocker(cur);

		if (blocker != NULL) {
		    cout << "Found blocker\n";

			if (cur->parent() == NULL || blocker == cur) {
				double value = cur->default_move().value;
				cur->lower_bound(value);
				cur->upper_bound(value);
				cur->utility_upper_bound = value;
			} else {
				const map<OBS_TYPE, VNode*>& siblings =
					cur->parent()->children();
				for (map<OBS_TYPE, VNode*>::const_iterator it = siblings.begin();
					it != siblings.end(); it++) {
					VNode* node = it->second;
					double value = node->default_move().value;
					node->lower_bound(value);
					node->upper_bound(value);
					node->utility_upper_bound = value;
				}
			}

			Backup(cur);

			if (cur->parent() == NULL) {
				cur = NULL;
			} else {
				cur = cur->parent()->parent();
			}
		} else {
			break;
		}
	}
}

VNode* DESPOT::ConstructTree(vector<State*>& particles, RandomStreams& streams,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, History& history, double timeout,
	SearchStatistics* statistics) {

	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

	VNode* root = new VNode(particles);

	assert (root->parent_ == nullptr);

	logd
		<< "[DESPOT::ConstructTree] START - Initializing lower and upper bounds at the root node.";

    InitBounds(root, lower_bound, upper_bound, streams, history);

	logd
		<< "[DESPOT::ConstructTree] END - Initializing lower and upper bounds at the root node.";

    /*
	if (statistics != NULL) {
		statistics->initial_lb = root->lower_bound();
		statistics->initial_ub = root->upper_bound();
	}
	*/

    double used_time = 0;
    int num_trials = 0;
    int total_ms = 0;
    do {
    	double start = clock();
    	VNode* cur = Trial(root, streams, lower_bound, upper_bound, model, history, statistics);
    	used_time += double(clock() - start) / CLOCKS_PER_SEC;

    	start = clock();
    	Backup(cur);
    	if (statistics != NULL) {
    		statistics->time_backup += double(clock() - start) / CLOCKS_PER_SEC;
    	}
    	used_time += double(clock() - start) / CLOCKS_PER_SEC;

    	num_trials++;
    } while (used_time * (num_trials + 1.0) / num_trials < timeout
    	&& (root->upper_bound() - root->lower_bound()) > 1e-6);

    cout << "Finished tree construction after " << num_trials << "trials in " << total_ms << "ms, U:"
     << root->upper_bound() << " L:" << root->lower_bound() <<"\n";

    /*
	if (statistics != NULL) {
		statistics->num_particles_after_search = model->NumActiveParticles();
		statistics->num_policy_nodes = root->PolicyTreeSize();
		statistics->num_tree_nodes = root->Size();
		statistics->final_lb = root->lower_bound();
		statistics->final_ub = root->upper_bound();
		statistics->time_search = used_time;
		statistics->num_trials = num_trials;
	}
	*/

	return root;
}

void DESPOT::Compare() {
	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
	SearchStatistics statistics;

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	VNode* root = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, &statistics);

	CheckDESPOT(root, root->lower_bound());
	CheckDESPOTSTAR(root, root->lower_bound());
	delete root;
}

void DESPOT::InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
	RandomStreams& streams, History& history) {
	streams.position(vnode->depth());

	ValuedAction move = lower_bound->Value(vnode->particles(), streams, history);
	move.value *= Discount(vnode->depth());
	
	vnode->default_move(move);
	vnode->lower_bound(move.value);
}

void DESPOT::InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
	RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	double upper = upper_bound->Value(vnode->particles(), streams, history);

	vnode->utility_upper_bound = upper * Discount(vnode->depth());
	upper = upper * Discount(vnode->depth()) - Globals::config.pruning_constant;

	vnode->upper_bound(upper);
}

void DESPOT::InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
	ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history) {
	InitLowerBound(vnode, lower_bound, streams, history);
	InitUpperBound(vnode, upper_bound, streams, history);
	if (vnode->upper_bound() < vnode->lower_bound()
		// close gap because no more search can be done on leaf node
		|| vnode->depth() == Globals::config.search_depth - 1) {
		vnode->upper_bound(vnode->lower_bound());
	}
}

void DESPOT::ImprovedPolicy(VNode* node){
    int num_children = node->children_.size();

    float* res = &improvedProbabilities[0];

    if(num_children > 3){
        cerr << "Improved Policy: Root has too many children: " << num_children << "\n";
        exit(-1);
    }

    res[0] = 0;
    res[1] = 0;
    res[2] = 0;

    // num actions: 3
	double values[3];
    double exp_values[3];
    double sum = 0;

    double maxVal = Globals::NEG_INFTY;

    double tau = 0.1;

    int actionIndex = 0;

    for (int action = 0; action < num_children; action++) {
        QNode* qnode = node->children_[action];
        double val = qnode->lower_bound();

        if(val < -2){
            val = -2;
        }

        if(val > 2){
            val = 2;
        }

        val = val / tau;

        if(val > maxVal){
            maxVal = val;
            actionIndex = action;
        }

        values[action] = val;
    }

    for (int action = 0; action < num_children; action++) {
        exp_values[action] = exp((values[action] - maxVal));
        sum += exp_values[action];
    }

    for (int action = 0; action < num_children; action++) {
        res[action] = exp_values[action] / sum;
    }


    //res[actionIndex] = 1;
}

ValuedAction DESPOT::Search() {
//	if (logging::level() >= logging::DEBUG) {
//		model_->PrintBelief(*belief_);
//	}
    DESPOT::call_counter = 0;

	if (Globals::config.silence != true) {
		model_->PrintBelief(*belief_);
	}

	if (Globals::config.time_per_move <= 0) // Return a random action if no time is allocated for planning
 		exit(-1);

	double start = get_time_second();
	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
	logi << "[DESPOT::Search] Time for sampling " << particles.size()
		<< " particles: " << (get_time_second() - start) << "s" << endl;
		
	if (Globals::config.silence != true) {
		model_->PrintParticles(particles);
	}

	statistics_ = SearchStatistics();

	start = get_time_second();
	static RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	LookaheadUpperBound* ub = dynamic_cast<LookaheadUpperBound*>(upper_bound_);
	if (ub != NULL) { // Avoid using new streams for LookaheadUpperBound
		static bool initialized = false;
		if (!initialized ) {
			lower_bound_->Init(streams);
			upper_bound_->Init(streams);
			initialized = true;
		}
	} else {
		streams = RandomStreams(Globals::config.num_scenarios,
			Globals::config.search_depth);
		lower_bound_->Init(streams);
		upper_bound_->Init(streams);
	}

	root_ = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, &statistics_);

	//logi << "[DESPOT::Search] Time for tree construction: "
	//	<< (get_time_second() - start) << "s" << endl;

    ImprovedPolicy(root_);
    ValuedAction astar = OptimalAction(root_);

	cout << "Root value pred: " << root_->initial_lower_bound_
	    << ", Probabilities: [" << improvedProbabilities[0]
	    << ", " << improvedProbabilities[1]
	    << ", " << improvedProbabilities[2]  << "]\n";

	start = get_time_second();
	root_->Free(*model_);
	//logi << "[DESPOT::Search] Time for freeing particles in search tree: "
	//	<< (get_time_second() - start) << "s" << endl;

	start = get_time_second();
	delete root_;

	//logi << "[DESPOT::Search] Time for deleting tree: "
	//	<< (get_time_second() - start) << "s" << endl;
	//logi << "[DESPOT::Search] Search statistics:" << endl << statistics_
	//	<< endl;

	return astar;
}

double DESPOT::CheckDESPOT(const VNode* vnode, double regularized_value) {
	cerr << "DESPOT::CheckDESPOT deleted\n";
	throw "DESPOT::CheckDESPOT deleted";
}

double DESPOT::CheckDESPOTSTAR(const VNode* vnode, double regularized_value) {
    cerr << "DESPOT::CheckDESPOTSTAR deleted\n";
	throw "DESPOT::CheckDESPOTSTAR deleted";
}

VNode* DESPOT::Prune(VNode* vnode, int& pruned_action, double& pruned_value) {
	vector<State*> empty;
	VNode* pruned_v = new VNode(empty, vnode->depth(), NULL,
		vnode->edge());

	vector<QNode*>& children = vnode->children();
	int astar = -1;
	double nustar = Globals::NEG_INFTY;
	QNode* qstar = NULL;
	for (int i = 0; i < children.size(); i++) {
		QNode* qnode = children[i];
		double nu;
		QNode* pruned_q = Prune(qnode, nu);

		if (nu > nustar) {
			nustar = nu;
			astar = qnode->edge();

			if (qstar != NULL) {
				delete qstar;
			}

			qstar = pruned_q;
		} else {
			delete pruned_q;
		}
	}

	if (nustar < vnode->default_move().value) {
		nustar = vnode->default_move().value;
		astar = vnode->default_move().action;
		delete qstar;
	} else {
		pruned_v->children().push_back(qstar);
		qstar->parent(pruned_v);
	}

	pruned_v->lower_bound(vnode->lower_bound()); // for debugging
	pruned_v->upper_bound(vnode->upper_bound());

	pruned_action = astar;
	pruned_value = nustar;

	return pruned_v;
}

QNode* DESPOT::Prune(QNode* qnode, double& pruned_value) {
	QNode* pruned_q = new QNode((VNode*) NULL, qnode->edge());
	pruned_value = qnode->step_reward - Globals::config.pruning_constant;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		int astar;
		double nu;
		VNode* pruned_v = Prune(it->second, astar, nu);
		if (nu == it->second->default_move().value) {
			delete pruned_v;
		} else {
			pruned_q->children()[it->first] = pruned_v;
			pruned_v->parent(pruned_q);
		}
		pruned_value += nu;
	}

	pruned_q->lower_bound(qnode->lower_bound()); // for debugging
	pruned_q->upper_bound(qnode->upper_bound()); // for debugging

	return pruned_q;
}

//for debugging
void DESPOT::OutputWeight(QNode* qnode)
{	
	double qnode_weight = 0;
	std::vector<double> vnode_weight;
	if(qnode == NULL) {
		cout<<"NULL"<<endl;
		return;
	}
	if(qnode->parent()->depth() > 5) return;
	else{
		map<OBS_TYPE, VNode*>& vnodes = qnode->children();
		for (map<OBS_TYPE, VNode*>::iterator it = vnodes.begin();
		it != vnodes.end(); it++){
			if(it -> second==NULL){
				cout<<"qnode is empty"<<endl;
				return;
			}
			OptimalAction2(it->second);
			qnode_weight+=it-> second->Weight();
		}
		cout<<"Qnode_depth: "<<qnode->parent()->depth();
		cout<<" weight: "<<qnode_weight<<endl;
	}

}

//for debugging
void DESPOT::OptimalAction2(VNode* vnode) {
	if(vnode->depth() > 5) return;
	ValuedAction astar(-1, Globals::NEG_INFTY);
	QNode * best_qnode = NULL;
	for (int action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);
		if (qnode->lower_bound() > astar.value) {
			astar = ValuedAction(action, qnode->lower_bound());
			best_qnode = qnode;
		}
	}

	if (vnode->default_move().value > astar.value) {
		astar = vnode->default_move();
		best_qnode = NULL;
	}

	double not_weighted_value = 0;
	if(vnode->Weight()!=0) not_weighted_value = astar.value / vnode->Weight();
	double not_discounted_not_weighted_value = not_weighted_value / pow(0.95,vnode->depth());

	cout<<"Vnode_depth: "<<vnode->depth()<<" action: "<<astar.action<<" value: "<<astar.value<<" not_w_v: "<<not_weighted_value<<" not_w_d_v: "<<not_discounted_not_weighted_value<<" weight: "<<vnode->Weight()<<" #particle: "<<vnode->particle_num()<<endl;
	
	DESPOT::OutputWeight(best_qnode);

	return;
}


ValuedAction DESPOT::OptimalAction(VNode* vnode) {
    int num_children = vnode->children_.size();

	ValuedAction astar(-1, Globals::NEG_INFTY);

	cout << "Root action values: [";

	for (int action = 0; action < num_children; action++) {
		QNode* qnode = vnode->children_[action];
		if (qnode->lower_bound() > astar.value) {
			astar = ValuedAction(action, qnode->lower_bound());
		}
		cout << qnode->lower_bound() << ", ";
	}

	cout << "]\n";

	if (vnode->default_move().value > astar.value) {
		astar = vnode->default_move();
	}

	if(astar.action == -1){
        cerr << "Astar.action == -1\n";
        exit(-1);
	}

    /*
	if(true){
        astar.action = vnode->network_action;
        cout << "USE NETWORK ACTION!\n";
    }
    */

	return astar;
}

double DESPOT::Gap(VNode* vnode) {
	return (vnode->upper_bound() - vnode->lower_bound());
}

double DESPOT::WEU(VNode* vnode) {
	return WEU(vnode, Globals::config.xi);
}

// Can pass root as an argument, but will not affect performance much
double DESPOT::WEU(VNode* vnode, double xi) {
	VNode* root = vnode;
	while (root->parent() != NULL) {
		root = root->parent()->parent();
	}
	return Gap(vnode) - xi * vnode->Weight() * Gap(root);
}

VNode* DESPOT::SelectBestWEUNode(QNode* qnode) {
	double weustar = Globals::NEG_INFTY;
	VNode* vstar = nullptr;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;

        if(!vnode->done_){
            double weu = WEU(vnode);
            if (weu >= weustar) {
            	weustar = weu;
            	vstar = vnode->vstar;
            }
        }
	}
	return vstar;
}

QNode* DESPOT::SelectBestUpperBoundNode(VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (int action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		if (!qnode->done_ && qnode->upper_bound() > upperstar) {
			upperstar = qnode->upper_bound();
			astar = action;
		}
	}

	if(astar < 0){
	    cerr << "Astar: " << astar << "\n";
	    cerr << "Children: " << vnode->children().size() << "\n";
	    cerr << vnode->Child(0)->upper_bound() << "\n";
	}

	assert(astar >= 0);
	return vnode->Child(astar);
}

void DESPOT::Update(VNode* vnode) {
	if (vnode->IsLeaf()) {
		return;
	}

	double lower = vnode->default_move().value;
	double upper = vnode->default_move().value;
	double utility_upper = Globals::NEG_INFTY;

	for (int action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
		utility_upper = max(utility_upper, qnode->utility_upper_bound);
	}

	if (lower > vnode->lower_bound()) {
		vnode->lower_bound(lower);
	}
	if (upper < vnode->upper_bound()) {
		vnode->upper_bound(upper);
	}
	if (utility_upper < vnode->utility_upper_bound) {
		vnode->utility_upper_bound = utility_upper;
	}
}

void DESPOT::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;
	double utility_upper = qnode->step_reward
		+ Globals::config.pruning_constant;

	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;

		lower += vnode->lower_bound();
		upper += vnode->upper_bound();
		utility_upper += vnode->utility_upper_bound;
	}

	if (lower > qnode->lower_bound()) {
		qnode->lower_bound(lower);
	}
	if (upper < qnode->upper_bound()) {
		qnode->upper_bound(upper);
	}
	if (utility_upper < qnode->utility_upper_bound) {
		qnode->utility_upper_bound = utility_upper;
	}
}

void DESPOT::Backup(VNode* vnode) {
	//int iter = 0;
	//logd << "- Backup " << vnode << " at depth " << vnode->depth() << endl;
	while (true) {
		//logd << " Iter " << iter << " " << vnode << endl;

        Update(vnode);

        QNode* parentq = vnode->parent();
        if (parentq == NULL) {
        	break;
        }

        Update(parentq);

		//logd << " Updated Q-node to (" << parentq->lower_bound() << ", "
		//	<< parentq->upper_bound() << ")" << endl;

		vnode = parentq->parent();
		//iter++;
	}
	//logd << "* Backup complete!" << endl;
}

VNode* DESPOT::FindBlocker(VNode* vnode) {
	VNode* cur = vnode;
	int count = 1;
	while (cur != NULL) {
		if (cur->utility_upper_bound - count * Globals::config.pruning_constant
			<= cur->default_move().value) {
			break;
		}
		count++;
		if (cur->parent() == NULL) {
			cur = NULL;
		} else {
			cur = cur->parent()->parent();
		}
	}
	return cur;
}

void DESPOT::Expand(VNode* vnode,
	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
	const DSPOMDP* model, RandomStreams& streams,
	History& history) {
	vector<QNode*>& children = vnode->children();

	//cout << "Expanding V-Node " << vnode << ". Creating " << model->NumActions() << " children.\n";
    for (int action = 0; action < model->NumActions(); action++) {
    	QNode* qnode = new QNode(vnode, action);
    	children.push_back(qnode);

    	Expand(qnode, lower_bound, upper_bound, model, streams, history);
    }
}

void DESPOT::Expand(QNode* qnode, ScenarioLowerBound* lb,
	ScenarioUpperBound* ub, const DSPOMDP* model,
	RandomStreams& streams,
	History& history) {
	VNode* parent = qnode->parent();
	streams.position(parent->depth());
	map<OBS_TYPE, VNode*>& children = qnode->children();

	const vector<State*>& particles = parent->particles();

	double step_reward = 0;

	//compute the weight of the parent
	double parent_weight = parent -> Weight();

	// Partition particles by observation
	map<OBS_TYPE, vector<State*> > partitions;
	OBS_TYPE obs;
	double reward;
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		logd << " Original: " << *particle << endl;

		State* copy = model->Copy(particle);

		logd << " Before step: " << *copy << endl;

		bool terminal;
		if(Globals::config.no_importance_sampling == true)
			terminal = model->Step(*copy, streams.Entry(copy->scenario_id), qnode->edge(), reward, obs);
		else terminal = model->ImportanceSamplingStep(*copy, streams.Entry(copy->scenario_id),
			qnode->edge(), reward, obs);

		step_reward += reward * particle->weight;

		logd << " After step: " << *copy << " " << (reward * copy->weight)
			<< " " << reward << " " << copy->weight << endl;

		if (!terminal) {
			partitions[obs].push_back(copy);
		} else {
			model->Free(copy);
		}
	}

	//compute the weight of the particles for qnode, i.e., the weight of the children
	double children_weight = 0;
	for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
		it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;
		children_weight += partitions[obs][0]->Weight(partitions[obs]);
	}

	double normization_constant;
	if(Globals::config.unnormalized == true) normization_constant = 1;
	else normization_constant = (children_weight==0)? 1:parent_weight/children_weight;


	for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
		it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;

		for(int i=0; i<partitions[obs].size(); i++){
			partitions[obs][i]->weight = partitions[obs][i]->weight * normization_constant;
		}

	}

	step_reward = Discount(parent->depth()) * step_reward
		- Globals::config.pruning_constant;//pruning_constant is used for regularization

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	// Create new belief nodes
	for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
		it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;
		logd << " Creating node for obs " << obs << endl;
		VNode* vnode = new VNode(partitions[obs], parent->depth() + 1, qnode, obs);
		logd << " New node created!" << endl;
		children[obs] = vnode;

		history.Add(qnode->edge(), obs);
		InitBounds(vnode, lb, ub, streams, history);
		history.RemoveLast();
		logd << " New node's bounds: (" << vnode->lower_bound() << ", "
			<< vnode->upper_bound() << ")" << endl;

		lower_bound += vnode->lower_bound();
		upper_bound += vnode->upper_bound();
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->utility_upper_bound = upper_bound + Globals::config.pruning_constant;

	qnode->default_value = lower_bound; // for debugging
}

ValuedAction DESPOT::Evaluate(VNode* root, vector<State*>& particles,
	RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model) {
	double value = 0;

	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		VNode* cur = root;
		State* copy = model->Copy(particle);
		double discount = 1.0;
		double val = 0;
		int steps = 0;

		while (!streams.Exhausted()) {
			int action =
				(cur != NULL) ?
					OptimalAction(cur).action : prior->GetAction(*copy);

			assert(action != -1);

			double reward;
			OBS_TYPE obs;
			bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
				action, reward, obs);

			val += discount * reward;
			discount *= Discount();

			if (!terminal) {
				prior->Add(action, obs);
				streams.Advance();
				steps++;

				if (cur != NULL && !cur->IsLeaf()) {
					QNode* qnode = cur->Child(action);
					map<OBS_TYPE, VNode*>& vnodes = qnode->children();
					cur = vnodes.find(obs) != vnodes.end() ? vnodes[obs] : NULL;
				}
			} else {
				break;
			}
		}

		for (int i = 0; i < steps; i++) {
			streams.Back();
			prior->PopLast();
		}

		model->Free(copy);

		value += val;
	}

	return ValuedAction(OptimalAction(root).action, value / particles.size());
}

void DESPOT::belief(Belief* b) {
	logi << "[DESPOT::belief] Start: Set initial belief." << endl;
	belief_ = b;
	history_.Truncate(0);

	lower_bound_->belief(b); // needed for POMCPScenarioLowerBound
	logi << "[DESPOT::belief] End: Set initial belief." << endl;
}

void DESPOT::Update(int action, OBS_TYPE obs) {
    cout << "DESPOT: int action, OBS_TYPE obs\n";
    exit(-1);

	double start = get_time_second();

	belief_->Update(action, obs);
	history_.Add(action, obs);

	lower_bound_->belief(belief_);

	logi << "[Solver::Update] Updated belief, history and root with action "
		<< action << ", observation " << obs
		<< " in " << (get_time_second() - start) << "s" << endl;
}
