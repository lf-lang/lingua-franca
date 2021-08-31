#pragma once

#include <vector>
#include <cstddef>

// global variables, initialized in LF
extern size_t NUM_WORKERS;
extern size_t SIZE;
extern size_t THRESHOLD;
extern size_t PRIORITIES;
extern size_t SOLUTIONS_LIMIT;

extern const size_t SOLUTIONS[];

bool boardValid(size_t n, const std::vector<size_t>& a);
size_t nqueensKernelSeq(const std::vector<size_t>& a, size_t depth, size_t size);

struct WorkMessage {

	size_t priority;
	std::vector<size_t> data;
	size_t depth;

	WorkMessage(size_t _priority, const std::vector<size_t>& _data, size_t _depth);
};

enum MsgType {
	StopMsg,
	ResultMsg,
	DoneMsg
};

struct Message {

	MsgType type;
	size_t numResults;
};
