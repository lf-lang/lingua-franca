#pragma once

#include <cmath>

#include "GridNode.h"

// global variables, need initialization

//FIXME: this needs to change
extern std::map<int,GridNode*> * allNodes;

extern int PRIORITY_GRANULARITY;
extern int NUM_WORKERS;
extern int GRID_SIZE;
extern int PRIORITIES;
extern int THRESHOLD;

int calcPriority(GridNode * gridNode);
void initializeData();
GridNode * originNode();
GridNode * targetNode();
bool validate();
void busyWait();
int calcPriority(GridNode * gridNode);

struct WorkMessage {

	GridNode * node;
	GridNode * target;
	int priority;

	WorkMessage(GridNode * node, GridNode * target);
};

enum MsgType {
	ReceivedMsg,
	DoneMsg,
	StopMsg
};

struct Message {

	MsgType type;
};