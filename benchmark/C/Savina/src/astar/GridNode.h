#pragma once

#include <vector>
#include <atomic>

class GridNode {
public:
	int id;
	int i;
	int j;
	int k;

	vector<GridNode*> neighbors;
	atomic<GridNode*> parentInPath;

	//fields used in computing distance
	int distanceFromRoot;

	GridNode(int id, int i, int j, int k);

	GridNode(GridNode& node) = default;
	GridNode(GridNode&& node) = default;

	int numNeighbors();

	GridNode * neighbor(const int n);

	bool setParent(GridNode * node);

	double distanceFrom(const GridNode * node);

	bool addNeighbor(GridNode * node);
};