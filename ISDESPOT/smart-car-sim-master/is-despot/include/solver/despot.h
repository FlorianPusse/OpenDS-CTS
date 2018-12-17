#ifndef DESPOT_H
#define DESPOT_H

#include "core/solver.h"
#include "core/pomdp.h"
#include "core/belief.h"
#include "core/node.h"
#include "core/globals.h"
#include "core/history.h"
#include "random_streams.h"
#include "../problems/hybridVisual_car/src/connector.h"
#include "../problems/hybridVisual_car/src/Path.h"
#include <mutex>

#define NUM_PEDESTRIANS 4

class DESPOT: public Solver {
friend class VNode;

protected:
	SearchStatistics statistics_;

	ScenarioLowerBound* lower_bound_;
	ScenarioUpperBound* upper_bound_;

	static python_connector* conn;

public:
	DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);
	DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, python_connector* connector, Belief* belief = NULL);
	virtual ~DESPOT();

    static mutex* lock;
    static Path* path;
    static int call_counter;
    static float initial_history[history_size + 4 + 2*NUM_PEDESTRIANS];

    VNode* root_;

	ValuedAction Search();
	float improvedProbabilities[3];

	void belief(Belief* b);
	void Update(int action, OBS_TYPE obs);

	ScenarioLowerBound* lower_bound() const;
	ScenarioUpperBound* upper_bound() const;

	static VNode* ConstructTree(vector<State*>& particles, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, double timeout,
		SearchStatistics* statistics = NULL);

    void ImprovedPolicy(VNode* node);

    static void Expand(VNode* vnode,
        	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
        	const DSPOMDP* model, RandomStreams& streams, History& history);
    static void Expand(QNode* qnode, ScenarioLowerBound* lower_bound,
    		ScenarioUpperBound* upper_bound, const DSPOMDP* model,
    		RandomStreams& streams, History& history);

    static void TreeThread(int thread_number, double timeout, VNode* root, RandomStreams streams,
        ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
        const DSPOMDP* model, History history, int& trials);

    static VNode* ParallelTrial(VNode* root, RandomStreams& streams,
        ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
        const DSPOMDP* model, History& history, int thread_number,
        vector<VNode*>& ex_newNodes, vector<State*>& ex_states, vector<float*>& ex_newHistories,
        vector<ValuedAction>& ex_valuedActions);

    static void ParallelExpand(VNode* vnode,
                       	ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
                       	const DSPOMDP* model, RandomStreams& streams,
                       	History& history, int thread_number,
                       	vector<VNode*>& ex_newNodes, vector<State*>& ex_states,
                       	vector<float*>& ex_newHistories, vector<ValuedAction>& ex_valuedActions);

    static void ExpandPartOne(QNode* qnode, const DSPOMDP* model,
                              	RandomStreams& streams, map<OBS_TYPE, vector<State*> >& partitions, double& step_reward, int& num_elements,
                              	vector<VNode*>& ex_newNodes, vector<State*>& ex_states, vector<float*>& ex_newHistories, int thread_number);
    static void ExpandPartTwo(QNode* qnode, double step_reward, double num_nodes,
                                  ScenarioLowerBound* lb, ScenarioUpperBound* ub, RandomStreams& streams, History& history, int offset,
                                  vector<VNode*>& ex_newNodes, vector<ValuedAction>& ex_valuedActions);

protected:
	static VNode* Trial(VNode* root, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, SearchStatistics* statistics =
			NULL);
	static void InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
		RandomStreams& streams, History& history);

	static void InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
		RandomStreams& streams, History& history);
	static void InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
        RandomStreams& streams, History& history, ValuedAction valuedAction);
	static void InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history);
	static void InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
        ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history, ValuedAction valuedAction);
	static void Backup(VNode* vnode);

	static double Gap(VNode* vnode);

	double CheckDESPOT(const VNode* vnode, double regularized_value);
	double CheckDESPOTSTAR(const VNode* vnode, double regularized_value);
	void Compare();

	static void ExploitBlockers(VNode* vnode);
	static VNode* FindBlocker(VNode* vnode);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode);
	static VNode* Prune(VNode* vnode, int& pruned_action, double& pruned_value);
	static QNode* Prune(QNode* qnode, double& pruned_value);
	static double WEU(VNode* vnode);
	static double WEU(VNode* vnode, double epsilon);
	static VNode* SelectBestWEUNode(QNode* qnode);
	static QNode* SelectBestUpperBoundNode(VNode* vnode);
	static ValuedAction OptimalAction(VNode* vnode);
	static void OptimalAction2(VNode* vnode);

	static ValuedAction Evaluate(VNode* root, vector<State*>& particles,
		RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model);

	//output the total weight of qnode, used for debugging
	static void OutputWeight(QNode* qnode);
};

#endif
