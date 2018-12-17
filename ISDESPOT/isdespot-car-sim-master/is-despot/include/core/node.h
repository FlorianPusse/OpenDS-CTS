#ifndef NODE_H
#define NODE_H

#include "pomdp.h"
#include "util/util.h"
#include "random_streams.h"
#include "util/logging.h"

#define NUM_PEDESTRIANS 4

class QNode;

/* =============================================================================
 * VNode class
 * =============================================================================*/

/**
 * A belief/value/AND node in the search tree.
 */
class VNode {
protected:
	Belief* belief_; // Used in AEMS
	int depth_;

	ValuedAction default_move_; // Value and action given by default policy
	double upper_bound_;

	// For POMCP
	int count_; // Number of visits on the node
	double value_; // Value of the node

    //PomdpState position;
public:
	bool done_ = false;
	QNode* parent_;
	vector<State*> particles_; // Used in DESPOT
	double lower_bound_;
	OBS_TYPE edge_;
    vector<QNode*> children_;

    double initial_lower_bound_;
    int network_action;

    // the history that lead to this node, with this NODE ALREADY INTEGRATED, space reserved for action and value
    float current_hist[history_size + 4 + 2*NUM_PEDESTRIANS];

	VNode* vstar;
	double likelihood; // Used in AEMS
	double utility_upper_bound;

	VNode(vector<State*>& particles, int depth = 0, QNode* parent = NULL, OBS_TYPE edge = -1);
	VNode(Belief* belief, int depth = 0, QNode* parent = NULL, OBS_TYPE edge = -1);
	VNode(int count, double value, int depth = 0, QNode* parent = NULL, OBS_TYPE edge = -1);
	~VNode();

	Belief* belief() const;
	const vector<State*>& particles() const;
	void depth(int d);
	int depth() const;
	void parent(QNode* parent);
	QNode* parent();
	OBS_TYPE edge();

	double Weight() const;

	const vector<QNode*>& children() const;
	vector<QNode*>& children();
	const QNode* Child(int action) const;
	QNode* Child(int action);
	int Size() const;
	int PolicyTreeSize() const;

	void default_move(ValuedAction move);
	ValuedAction default_move() const;
	void lower_bound(double value);
	double lower_bound() const;
	void upper_bound(double value);
	double upper_bound() const;

	bool IsLeaf();

	void Add(double val);
	void count(int c);
	int count() const;
	void value(double v);
	double value() const;

	void PrintTree(int depth = -1, ostream& os = cout);
	void PrintPolicyTree(int depth = -1, ostream& os = cout);

	void Free(const DSPOMDP& model);

	int particle_num();
};

/* =============================================================================
 * QNode class
 * =============================================================================*/

/**
 * A Q-node/AND-node (child of a belief node) of the search tree.
 */
class QNode {
protected:
	VNode* parent_;
	double lower_bound_;
	double upper_bound_;

	// For POMCP
	int count_; // Number of visits on the node
	double value_; // Value of the node

public:
	bool done_ = false;
	int edge_;
    map<OBS_TYPE, VNode*> children_;
	double default_value;
	double utility_upper_bound;
	double step_reward;
	double likelihood;
	VNode* vstar;

	QNode(VNode* parent, int edge);
	QNode(int count, double value);
	~QNode();

	void parent(VNode* parent);
	VNode* parent();
	int edge();
	map<OBS_TYPE, VNode*>& children();
	VNode* Child(OBS_TYPE obs);
	int Size() const;
	int PolicyTreeSize() const;

	double Weight() const;

	void lower_bound(double value);
	double lower_bound() const;
	void upper_bound(double value);
	double upper_bound() const;

	void Add(double val);
	void count(int c);
	int count() const;
	void value(double v);
	double value() const;
};

#endif
