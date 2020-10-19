
#include "GridNode.hh"
#include <cmath>
#include <algorithm>


GridNode::GridNode(int id, int i, int j, int k):
id(id), i(i), j(j), k(k) {

	distanceFromRoot = id == 0 ? 0 : -1;
}


int GridNode::numNeighbors() {
	return neighbors.size();
}

GridNode * GridNode::neighbor(const int n) {
	return neighbors[n];
}

bool GridNode::setParent(GridNode * node) {
	GridNode * tmp = nullptr;
	const bool success = parentInPath.compare_exchange_strong(tmp, node);
	if(success) {
		distanceFromRoot = distanceFromRoot + static_cast<int>(distanceFrom(node));
	}
	return success;
}

double GridNode::distanceFrom(const GridNode * node) {
	const int iDiff = i - node->i;
	const int jDiff = j - node->j;
	const int kDiff = k - node->k;
	return sqrt((iDiff * iDiff) + (jDiff * jDiff) + (kDiff * kDiff));
}

bool GridNode::addNeighbor(GridNode * node) {
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
