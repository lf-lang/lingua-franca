// Copyright (C) 2020 TU Dresden

#include "GuidedSearchCommon.hh"


size_t Grid::calcPriority(const GridNode* gridNode) const {
    int distDiff = std::max(gridSize - gridNode->i, std::max(gridSize - gridNode->j, gridSize - gridNode->k));
    int priorityUnit = distDiff / 8; // 8 represents a hardcoded priority granularity (same as in the akka version)
    int resultPriority = abs((double)(priorities - priorityUnit));
    return resultPriority;
}

void Grid::initializeData() {
    allNodes.clear();
    size_t id = 0;
    for(size_t i{0}; i < gridSize; i++) {
        for (size_t j{0}; j < gridSize; j++) {
            for (size_t k{0}; k < gridSize; k++) {
                allNodes.try_emplace(id, id, i, j, k);
                id++;
            }
        }
    }

    srand(123456);
    for(size_t j{0}; j < allNodes.size(); j++) {
        GridNode* gridNode = &allNodes.at(j);

        size_t iterCount{0};
        size_t neighborCount{0};

        for (size_t i{0}; i < 2; i++) {
            for (size_t j{0}; j < 2; j++) {
                for (size_t k{0}; k < 2; k++) {

                    iterCount++;
                    if(iterCount == 1 || iterCount == 8) {
                        continue;
                    }

                    const bool addNeighbor = (iterCount == 7 && neighborCount == 0) || (static_cast<int>(rand() % 2));
                    if(addNeighbor) {
                        const int newI = std::min(gridSize - 1, gridNode->i + i);
                        const int newJ = std::min(gridSize - 1, gridNode->j + j);
                        const int newK = std::min(gridSize - 1, gridNode->k + k);

                        const int newId = (gridSize * gridSize * newI) + (gridSize * newJ) + newK;
                        GridNode* newNode = &allNodes.at(newId);

                        if(gridNode->addNeighbor(newNode)) {
                            neighborCount++;
                        }
                    }
                }
            }
        }
    }
}

GridNode* Grid::targetNode() {
    const int axisVal = static_cast<int>(0.80 * gridSize);
    const int targetId = (axisVal * gridSize * gridSize) + (axisVal * gridSize) + axisVal;
    return &allNodes.at(targetId);
}

const GridNode* Grid::targetNode() const {
    const int axisVal = static_cast<int>(0.80 * gridSize);
    const int targetId = (axisVal * gridSize * gridSize) + (axisVal * gridSize) + axisVal;
    return &allNodes.at(targetId);
}

bool Grid::validate() const {
    const GridNode* parentNode = targetNode();
    while (parentNode->parent() != nullptr) {
        parentNode = parentNode->parent();
    }
    return parentNode == originNode();
}
