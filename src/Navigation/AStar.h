//
// Created by User on 11/04/2026.
//

#ifndef DOGFIGHT_ASTAR_H
#define DOGFIGHT_ASTAR_H


#include <cmath>
#include <vector>
#include <queue>
#include "../World/GraphBuilder.h"
#include <unordered_map>


class NavigationGraph;

class AStar {
public:
    struct NodeSearchData {
        float gScore = INFINITY;
        int parentIdx = -1;
    };

    struct PQElement {
        float fScore;
        int nodeIdx;
        bool operator>(const PQElement& other) const { return fScore > other.fScore; }
    };

    explicit AStar(int maxNodesHint = 10000);
    ~AStar() = default;

    std::vector<Vector3> FindPath(NavigationGraph& graph, int startIdx, int targetIdx);

private:
    void Reset(int nodesCount);

    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<>> m_openSet;
    std::vector<NodeSearchData> m_searchData;
};

#endif //DOGFIGHT_ASTAR_H