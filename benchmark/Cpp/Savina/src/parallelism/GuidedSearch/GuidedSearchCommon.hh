#pragma once

#include <map>
#include <cmath>

#include "GridNode.hh"

class Grid {
  private:
	const size_t gridSize;
	const size_t priorities;

	std::map<size_t, GridNode> allNodes;
  public:
	Grid(size_t gridSize, size_t priorities)
	  : gridSize(gridSize), priorities(priorities) {}

	void initializeData();
	size_t calcPriority(const GridNode* gridNode) const;
	GridNode* originNode() { return &allNodes.at(0); }
	const GridNode* originNode() const { return &allNodes.at(0); }
	GridNode* targetNode();
	const GridNode* targetNode() const;
	bool validate() const;

	size_t calcPriority(GridNode* gridNode) const;
};

void busyWait();

struct WorkMessage {
	GridNode* node;
	GridNode* target;

	WorkMessage(GridNode* node, GridNode* target)
	  : node{node}, target{target} {}
};

enum MsgType {
	ReceivedMsg,
	DoneMsg,
	StopMsg
};

struct Message {
	MsgType type;
};
