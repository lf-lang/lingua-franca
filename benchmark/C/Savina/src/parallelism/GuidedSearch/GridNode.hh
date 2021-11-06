// Copyright (C) 2020 TU Dresden

#pragma once

#include <vector>
#include <atomic>
#include <cstddef>
#include <ostream>

class GridNode {
public:
    const size_t id;
    const size_t i;
    const size_t j;
    const size_t k;

private:
    std::vector<GridNode*> neighbors;
    std::atomic<GridNode*> parentInPath;

public:
    GridNode(size_t id, size_t i, size_t j, size_t k)
      : id(id), i(i), j(j), k(k), parentInPath{nullptr} {}

    size_t numNeighbors() const { return neighbors.size(); };
    GridNode* neighbor(size_t n) const { return neighbors[n]; };
    const GridNode* parent() const { return parentInPath.load(std::memory_order_acquire); }

    bool setParent(GridNode * node);

    double distanceFrom(const GridNode * node);

    bool addNeighbor(GridNode * node);

    /**
     * Write a string representation of this class to 'string_rep', which 
     * has length 'length'.
     * @return string_rep with the string representation filled out.
     */
    char* toString(char* string_rep, size_t length);
};

std::ostream& operator<<(std::ostream& os, const GridNode& g);
