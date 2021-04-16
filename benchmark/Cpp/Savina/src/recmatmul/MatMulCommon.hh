#pragma once

#include <vector>

extern std::vector<std::vector<double>> * A;
extern std::vector<std::vector<double>> * B;
extern std::vector<std::vector<double>> * C;

void initializeData(int dataLength);
bool valid(int dataLength);

struct WorkMessage {

	int priority;
	int srA; // srA = start row in matrix A
	int scA; // scA = start column in matrix A
	int srB;
	int scB;
	int srC;
	int scC;
	int numBlocks; // total number of elements per block in both dimensions
	int dim; // number of elements in one dimension in one block

	WorkMessage(int _priority, int _srA, int _scA, int _srB,
			int _scB, int _srC, int _scC, int _numBlocks, int _dim);
};

enum MsgType {
	StopMsg,
	DoneMsg
};

struct Message {

	MsgType type;
};
