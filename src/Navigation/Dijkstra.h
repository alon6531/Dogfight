//
// Created by User on 11/04/2026.
//

#ifndef DOGFIGHT_DIJKSTRA_H
#define DOGFIGHT_DIJKSTRA_H


#include <vector>
#include <queue>
#include <limits>

class NavigationGraph;

class Dijkstra {
public:

    struct PQElement {
        float distance;
        int nodeIdx;
        bool operator>(const PQElement& other) const { return distance > other.distance; }
    };

    Dijkstra(int maxNodesHint = 10000);
    ~Dijkstra() = default;


    void Compute(NavigationGraph& graph, int startNodeIdx, std::vector<float>& outDistances);

private:
    void PrepareStructures(int nodesCount);


    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<>> m_pq;
};

#endif //DOGFIGHT_DIJKSTRA_H