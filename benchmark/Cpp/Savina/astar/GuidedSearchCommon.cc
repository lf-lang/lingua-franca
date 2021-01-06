
#include "GuidedSearchCommon.hh"


std::map<int,GridNode*> * allNodes = nullptr; // not part of the reactor

// default values given here, but actual values are given my command line parameters
int PRIORITY_GRANULARITY = 8;
int NUM_WORKERS = 20;
int GRID_SIZE = 30;
int PRIORITIES = 30;
int THRESHOLD = 1024;

int calcPriority(GridNode * gridNode) {
	int availablePriorities = PRIORITIES;
	int distDiff = std::max(GRID_SIZE - gridNode->i, std::max(GRID_SIZE - gridNode->j, GRID_SIZE - gridNode->k));
	int priorityUnit = distDiff / PRIORITY_GRANULARITY;
	int resultPriority = abs(availablePriorities - priorityUnit);
	return resultPriority;
}

WorkMessage::WorkMessage(GridNode * node, GridNode * target):
				node(node), target(target) {
	this->priority = calcPriority(node);
}

void initializeData() {

	if(allNodes == nullptr) {
		allNodes = new std::map<int,GridNode*>();
		int id = 0;

		for(int i = 0; i < GRID_SIZE; i++) {
			for (int j = 0; j < GRID_SIZE; j++) {
				for (int k = 0; k < GRID_SIZE; k++) {
					GridNode * node = new GridNode(id, i, j, k);
					allNodes->emplace(id, node);
					++id;
				}
			}
		}

		srand(123456);
		for(int j = 0; j < allNodes->size(); j++) {
			GridNode * gridNode = allNodes->operator[](j);

			int iterCount = 0;
			int neighborCount = 0;

			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {

						++iterCount;
						if(iterCount == 1 || iterCount == 8) {
							continue;
						}

						const bool addNeighbor = (iterCount == 7 && neighborCount == 0) || (static_cast<int>(rand() % 2));
						if(addNeighbor) {
							const int newI = std::min(GRID_SIZE - 1, gridNode->i + i);
							const int newJ = std::min(GRID_SIZE - 1, gridNode->j + j);
							const int newK = std::min(GRID_SIZE - 1, gridNode->k + k);

							const int newId = (GRID_SIZE * GRID_SIZE * newI) + (GRID_SIZE * newJ) + newK;
							GridNode * newNode = allNodes->operator[](newId);

							if(gridNode->addNeighbor(newNode)) {
								++neighborCount;
							}
						}
					}
				}
			}
		}
	}

	// clear distance and parent values
	for(int j = 0; j < allNodes->size(); j++) {
		GridNode * gridNode = allNodes->operator[](j);

		gridNode->distanceFromRoot = gridNode->id == 0 ? 0 : -1;
		gridNode->parentInPath.store(nullptr);
	}
}

GridNode * originNode() {
	return allNodes->operator[](0);
}

GridNode * targetNode() {
	const int axisVal = static_cast<int>(0.80 * GRID_SIZE);
	const int targetId = (axisVal * GRID_SIZE * GRID_SIZE) + (axisVal * GRID_SIZE) + axisVal;
	GridNode * gridNode = allNodes->operator[](targetId);
	return gridNode;
}

bool validate() {

	GridNode * parentNode = targetNode();
	while (parentNode->parentInPath.load() != nullptr) {
		parentNode = parentNode->parentInPath.load();
	}
	GridNode * rootNode = originNode();
	return (parentNode == rootNode);
}



void busyWait() {
	for (int i = 0; i < 100; i++) {
		rand();
	}
}
