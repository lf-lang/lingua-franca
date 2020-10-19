#pragma once

#include <vector>

// global variables, initialized in LF
extern int NUM_WORKERS;
extern int SIZE;
extern int THRESHOLD;
extern int PRIORITIES;
extern int SOLUTIONS_LIMIT;

extern const long SOLUTIONS[];

bool boardValid(int n, std::vector<int> a);
int nqueensKernelSeq(std::vector<int> a, int depth, int size);

struct WorkMessage {

	int priority;
	std::vector<int> data;
	int depth;

	WorkMessage(int _priority, std::vector<int> _data, int _depth);
};

enum MsgType {
	StopMsg,
	ResultMsg,
	DoneMsg
};

struct Message {

	MsgType type;
	int numResults;
};
