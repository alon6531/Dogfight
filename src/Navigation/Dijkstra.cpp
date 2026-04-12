// src/Navigation/Pathfinding/Dijkstra.cpp
#include "Dijkstra.h"
#include "../World/GraphBuilder.h"

Dijkstra::Dijkstra(int maxNodesHint) {

}

void Dijkstra::PrepareStructures(int nodesCount) {
    // ניקוי התור
    while (!m_pq.empty()) m_pq.pop();
}

void Dijkstra::Compute(NavigationGraph& graph, int startNodeIdx, std::vector<float>& outDistances) {
    const auto& nodes = graph.GetNodes();
    int nodesCount = (int)nodes.size();


    if ((int)outDistances.size() != nodesCount) {
        outDistances.resize(nodesCount);
    }
    std::fill(outDistances.begin(), outDistances.end(), std::numeric_limits<float>::max());


    if (startNodeIdx < 0 || startNodeIdx >= nodesCount) return;


    PrepareStructures(nodesCount);
    outDistances[startNodeIdx] = 0.0f;
    m_pq.push({0.0f, startNodeIdx});


    while (!m_pq.empty()) {
        float d = m_pq.top().distance;
        int u = m_pq.top().nodeIdx;
        m_pq.pop();


        if (d > outDistances[u]) continue;

        for (const auto& edge : nodes[u].neighbors) {
            int v = edge.target;
            float weight = edge.weight;

            if (outDistances[u] + weight < outDistances[v]) {
                outDistances[v] = outDistances[u] + weight;
                m_pq.push({outDistances[v], v});
            }
        }
    }


    for (float& d : outDistances) {
        if (d == std::numeric_limits<float>::max()) {
            d = -1.0f;
        }
    }
}