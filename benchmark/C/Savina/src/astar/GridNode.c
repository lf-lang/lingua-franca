#include "GridNode.h"
#include <cmath>
#include <algorithm>

GridNode(int id, int i, int j, int k):
id(id), i(i), j(j), k(k) {

	distanceFromRoot = id == 0 ? 0 : -1;
}


int numNeighbors() {
	return neighbors.size();
}

GridNode* neighbor(const int n) {
	return neighbors[n];
}

bool setParent(GridNode * node) {
	GridNode * tmp = NULL;
	const bool success = parentInPath.compare_exchange_strong(tmp, node);
	if(success) {
		distanceFromRoot = distanceFromRoot + static_cast<int>(distanceFrom(node));
	}
	return success;
}

double distanceFrom(const GridNode * node) {
	const int iDiff = i - node->i;
	const int jDiff = j - node->j;
	const int kDiff = k - node->k;
	return sqrt((iDiff * iDiff) + (jDiff * jDiff) + (kDiff * kDiff));
}

bool addNeighbor(GridNode * node) {
	if(node == this) {
		return false;
	}
	if(std::find(std::begin(neighbors), std::end(neighbors), node) ==
			std::end(neighbors)) {
		neighbors.push_back(node);
		return true;
	}
	return false;
}